"""Convert a NavSatFix stream in a ROS 2 bag to a local UTM/ENU CSV map."""

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import NavSatFix

from .geodesy import latlon_to_utm, yaw_to_azimuth_degrees
from .quality import horizontal_standard_deviation


CSV_FIELDS = [
    "stamp",
    "map_x",
    "map_y",
    "map_z",
    "yaw",
    "azimuth_deg",
    "utm_easting",
    "utm_northing",
    "utm_zone",
    "hemisphere",
    "origin_easting",
    "origin_northing",
    "origin_altitude",
    "latitude",
    "longitude",
    "altitude",
    "status",
    "covariance_type",
    "variance_east",
    "variance_north",
    "horizontal_stddev",
    "segment_id",
]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Convert sensor_msgs/NavSatFix from a ROS 2 bag into a map-frame "
            "CSV. map_x is local UTM east and map_y is local UTM north."
        )
    )
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory")
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--topic", default="/f9p/fix", help="NavSatFix topic")
    parser.add_argument(
        "--min-status",
        type=int,
        default=0,
        help="Minimum NavSatStatus value; this driver reports RTK FIXED as 2",
    )
    parser.add_argument(
        "--fixed-only",
        action="store_true",
        help="Keep only this u-blox driver's RTK FIXED NavSatFix status (2)",
    )
    parser.add_argument(
        "--max-horizontal-stddev",
        type=float,
        default=0.75,
        help="Maximum horizontal one-sigma uncertainty in metres",
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.03,
        help="Minimum map distance between retained points in metres",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=12.0,
        help="Reject GNSS jumps faster than this speed in m/s; <=0 disables",
    )
    parser.add_argument(
        "--segment-gap",
        type=float,
        default=1.0,
        help="Start a new path segment after this accepted-sample time gap",
    )
    parser.add_argument(
        "--origin-samples",
        type=int,
        default=30,
        help="Use the median of the first N retained points as map origin",
    )
    parser.add_argument(
        "--yaw-window",
        type=float,
        default=0.5,
        help="Distance window used to estimate CSV path orientation in metres",
    )
    return parser.parse_args(argv)


def message_stamp(message, bag_timestamp):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    return stamp if stamp > 0.0 else bag_timestamp * 1e-9


def _finite_coordinate(message):
    return (
        math.isfinite(message.latitude)
        and math.isfinite(message.longitude)
        and -80.0 <= message.latitude <= 84.0
        and -180.0 <= message.longitude <= 180.0
    )


def _path_yaws(samples, window_distance):
    if not samples:
        return []
    yaws = []
    last_yaw = 0.0
    for index, sample in enumerate(samples):
        segment = sample["segment_id"]
        lower = index
        upper = index
        while lower > 0 and samples[lower - 1]["segment_id"] == segment:
            candidate = samples[lower - 1]
            if math.hypot(
                sample["easting"] - candidate["easting"],
                sample["northing"] - candidate["northing"],
            ) >= window_distance:
                lower -= 1
                break
            lower -= 1
        while upper + 1 < len(samples) and samples[upper + 1]["segment_id"] == segment:
            candidate = samples[upper + 1]
            if math.hypot(
                candidate["easting"] - sample["easting"],
                candidate["northing"] - sample["northing"],
            ) >= window_distance:
                upper += 1
                break
            upper += 1
        delta_east = samples[upper]["easting"] - samples[lower]["easting"]
        delta_north = samples[upper]["northing"] - samples[lower]["northing"]
        if math.hypot(delta_east, delta_north) > 1e-6:
            last_yaw = math.atan2(delta_north, delta_east)
        yaws.append(last_yaw)
    return yaws


def convert(args):
    bag_path = Path(args.bag).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not bag_path.is_dir():
        raise RuntimeError(f"bag directory does not exist: {bag_path}")
    if args.origin_samples < 1:
        raise RuntimeError("--origin-samples must be at least 1")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic_types.get(args.topic) != "sensor_msgs/msg/NavSatFix":
        available = sorted(
            name
            for name, type_name in topic_types.items()
            if type_name == "sensor_msgs/msg/NavSatFix"
        )
        raise RuntimeError(
            f"NavSatFix topic {args.topic!r} not found; available={available}"
        )
    reader.set_filter(StorageFilter(topics=[args.topic]))

    samples = []
    status_counts = Counter()
    rejected = Counter()
    zone = None
    northern = None
    previous = None
    segment_id = 0

    while reader.has_next():
        _, serialized_data, bag_timestamp = reader.read_next()
        message = deserialize_message(serialized_data, NavSatFix)
        status_counts[message.status.status] += 1
        if not _finite_coordinate(message):
            rejected["coordinate"] += 1
            continue
        if args.fixed_only:
            if message.status.status != 2:
                rejected["status"] += 1
                continue
        elif message.status.status < args.min_status:
            rejected["status"] += 1
            continue

        horizontal_stddev = horizontal_standard_deviation(
            message.position_covariance, message.position_covariance_type
        )
        if horizontal_stddev > args.max_horizontal_stddev:
            rejected["covariance"] += 1
            continue

        easting, northing, used_zone, used_northern = latlon_to_utm(
            message.latitude, message.longitude, zone=zone
        )
        if zone is None:
            zone = used_zone
            northern = used_northern
        if used_northern != northern:
            raise RuntimeError("the bag crosses the equator and cannot use one UTM map")

        stamp = message_stamp(message, bag_timestamp)
        if previous is not None:
            delta_time = stamp - previous["stamp"]
            if delta_time <= 0.0:
                rejected["timestamp"] += 1
                continue
            distance = math.hypot(
                easting - previous["easting"], northing - previous["northing"]
            )
            if delta_time > args.segment_gap:
                segment_id += 1
            elif args.max_speed > 0.0 and distance / delta_time > args.max_speed:
                rejected["speed"] += 1
                continue
            if distance < args.min_distance:
                rejected["distance"] += 1
                continue

        altitude = message.altitude if math.isfinite(message.altitude) else 0.0
        sample = {
            "stamp": stamp,
            "easting": easting,
            "northing": northing,
            "altitude": altitude,
            "latitude": message.latitude,
            "longitude": message.longitude,
            "status": message.status.status,
            "covariance_type": message.position_covariance_type,
            "variance_east": message.position_covariance[0],
            "variance_north": message.position_covariance[4],
            "horizontal_stddev": horizontal_stddev,
            "segment_id": segment_id,
        }
        samples.append(sample)
        previous = sample

    if len(samples) < 2:
        raise RuntimeError("fewer than two GNSS samples passed the selected filters")

    origin_count = min(len(samples), args.origin_samples)
    origin_easting = statistics.median(
        sample["easting"] for sample in samples[:origin_count]
    )
    origin_northing = statistics.median(
        sample["northing"] for sample in samples[:origin_count]
    )
    origin_altitude = statistics.median(
        sample["altitude"] for sample in samples[:origin_count]
    )
    yaws = _path_yaws(samples, max(0.01, args.yaw_window))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sample, yaw in zip(samples, yaws):
            writer.writerow(
                {
                    "stamp": f'{sample["stamp"]:.9f}',
                    "map_x": f'{sample["easting"] - origin_easting:.6f}',
                    "map_y": f'{sample["northing"] - origin_northing:.6f}',
                    "map_z": f'{sample["altitude"] - origin_altitude:.6f}',
                    "yaw": f"{yaw:.9f}",
                    "azimuth_deg": f"{yaw_to_azimuth_degrees(yaw):.6f}",
                    "utm_easting": f'{sample["easting"]:.6f}',
                    "utm_northing": f'{sample["northing"]:.6f}',
                    "utm_zone": zone,
                    "hemisphere": "N" if northern else "S",
                    "origin_easting": f"{origin_easting:.6f}",
                    "origin_northing": f"{origin_northing:.6f}",
                    "origin_altitude": f"{origin_altitude:.6f}",
                    "latitude": f'{sample["latitude"]:.10f}',
                    "longitude": f'{sample["longitude"]:.10f}',
                    "altitude": f'{sample["altitude"]:.4f}',
                    "status": sample["status"],
                    "covariance_type": sample["covariance_type"],
                    "variance_east": f'{sample["variance_east"]:.9f}',
                    "variance_north": f'{sample["variance_north"]:.9f}',
                    "horizontal_stddev": f'{sample["horizontal_stddev"]:.6f}',
                    "segment_id": sample["segment_id"],
                }
            )

    track_length = 0.0
    for first, second in zip(samples, samples[1:]):
        if first["segment_id"] == second["segment_id"]:
            track_length += math.hypot(
                second["easting"] - first["easting"],
                second["northing"] - first["northing"],
            )
    metadata = {
        "format_version": 1,
        "source_bag": str(bag_path),
        "source_topic": args.topic,
        "coordinate_convention": (
            "REP-105 map; x=local UTM east, y=local UTM north, z=up"
        ),
        "utm_zone": zone,
        "hemisphere": "N" if northern else "S",
        "origin": {
            "easting": origin_easting,
            "northing": origin_northing,
            "altitude": origin_altitude,
            "sample_count": origin_count,
        },
        "filters": {
            "fixed_only": args.fixed_only,
            "min_status": args.min_status,
            "max_horizontal_stddev": args.max_horizontal_stddev,
            "min_distance": args.min_distance,
            "max_speed": args.max_speed,
            "segment_gap": args.segment_gap,
            "yaw_window": args.yaw_window,
        },
        "input_status_counts": dict(sorted(status_counts.items())),
        "rejected_counts": dict(sorted(rejected.items())),
        "output_points": len(samples),
        "segments": max(sample["segment_id"] for sample in samples) + 1,
        "track_length_m": track_length,
    }
    metadata_path = output_path.with_suffix(output_path.suffix + ".metadata.json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    return output_path, metadata_path, metadata


def main(argv=None):
    try:
        output_path, metadata_path, metadata = convert(parse_args(argv))
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f"ERROR: {error}") from error
    print(f"CSV: {output_path}")
    print(f"Metadata: {metadata_path}")
    print(
        f"UTM origin: zone {metadata['utm_zone']}{metadata['hemisphere']}, "
        f"E={metadata['origin']['easting']:.3f}, "
        f"N={metadata['origin']['northing']:.3f}"
    )
    print(
        f"Output: {metadata['output_points']} points, "
        f"{metadata['segments']} segments, {metadata['track_length_m']:.1f} m"
    )
    print(f"Input status: {metadata['input_status_counts']}")
    print(f"Rejected: {metadata['rejected_counts']}")


if __name__ == "__main__":
    main()
