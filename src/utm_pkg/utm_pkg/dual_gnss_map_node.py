"""Map-frame visualization and protected global yaw from two GNSS antennas."""

import csv
import math
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray

from .geodesy import (
    latlon_to_utm,
    shortest_angular_distance,
    yaw_quaternion,
    yaw_to_azimuth_degrees,
)
from .quality import circular_ema, horizontal_standard_deviation


@dataclass
class FixSample:
    stamp: float
    received_at: float
    x: float
    y: float
    altitude: float
    status: int
    covariance_type: int
    variance_east: float
    variance_north: float
    horizontal_stddev: float
    header_stamp: object

    @property
    def fixed(self):
        # This repository's ublox driver maps fixed carrier phase to status=2.
        return self.status == 2


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class DualGnssMapNode(Node):
    def __init__(self):
        super().__init__("dual_gnss_map")
        self._declare_parameters()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.rear_topic = str(self.get_parameter("rear_topic").value)
        self.front_topic = str(self.get_parameter("front_topic").value)
        self.require_rtk_fixed = bool(self.get_parameter("require_rtk_fixed").value)
        self.max_horizontal_stddev = float(
            self.get_parameter("max_horizontal_stddev_m").value
        )
        self.sync_tolerance = float(self.get_parameter("sync_tolerance_sec").value)
        self.max_interpolation_gap = float(
            self.get_parameter("max_interpolation_gap_sec").value
        )
        self.pair_wait = float(self.get_parameter("pair_wait_sec").value)
        self.minimum_baseline = float(self.get_parameter("minimum_baseline_m").value)
        self.maximum_baseline = float(self.get_parameter("maximum_baseline_m").value)
        self.max_yaw_rate = math.radians(
            float(self.get_parameter("maximum_yaw_rate_deg_s").value)
        )
        self.smoothing_alpha = float(
            self.get_parameter("heading_smoothing_alpha").value
        )
        self.heading_timeout = float(self.get_parameter("heading_timeout_sec").value)
        self.max_trail_points = int(self.get_parameter("max_trail_points").value)
        self.trail_min_distance = float(
            self.get_parameter("trail_min_distance_m").value
        )
        self.publish_trails = bool(self.get_parameter("publish_sensor_trails").value)

        self.reference_path, self.reference_marker = self._load_csv(
            str(self.get_parameter("csv_file").value)
        )
        self._create_interfaces()

        self.rear_samples = deque(maxlen=240)
        self.front_samples = deque(maxlen=120)
        self.rear_trail = deque(maxlen=max(2, self.max_trail_points))
        self.front_trail = deque(maxlen=max(2, self.max_trail_points))
        self.counts = Counter()
        self.last_rejection = "waiting_for_data"
        self.last_baseline = math.nan
        self.last_sync_delta = math.nan
        self.last_rear_stddev = math.inf
        self.last_front_stddev = math.inf
        self.last_raw_yaw = None
        self.last_raw_stamp = None
        self.smoothed_yaw = None
        self.last_valid_wall = None
        self.valid_state = False
        self.last_rear_input_stamp = None
        self.last_front_input_stamp = None

        self.publish_reference()
        self.process_timer = self.create_timer(0.01, self.process_pending)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.path_timer = self.create_timer(
            float(self.get_parameter("trail_publish_period_sec").value),
            self.publish_trails_callback,
        )
        self.get_logger().info(
            "Dual GNSS convention: F9P=rear, F9R=front, global_yaw=rear->front "
            "in REP-105 map coordinates. This node intentionally publishes no TF."
        )

    def _declare_parameters(self):
        self.declare_parameter("csv_file", "")
        self.declare_parameter("rear_topic", "/f9p/fix")
        self.declare_parameter("front_topic", "/f9r/fix")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("require_rtk_fixed", True)
        self.declare_parameter("max_horizontal_stddev_m", 0.25)
        self.declare_parameter("sync_tolerance_sec", 0.09)
        self.declare_parameter("max_interpolation_gap_sec", 0.12)
        self.declare_parameter("pair_wait_sec", 0.10)
        self.declare_parameter("minimum_baseline_m", 0.65)
        self.declare_parameter("maximum_baseline_m", 1.25)
        self.declare_parameter("maximum_yaw_rate_deg_s", 120.0)
        self.declare_parameter("heading_smoothing_alpha", 0.35)
        self.declare_parameter("heading_timeout_sec", 0.50)
        self.declare_parameter("publish_sensor_trails", True)
        self.declare_parameter("max_trail_points", 1000)
        self.declare_parameter("trail_min_distance_m", 0.05)
        self.declare_parameter("trail_publish_period_sec", 1.0)

    def _create_interfaces(self):
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        visualization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.reference_path_pub = self.create_publisher(
            PathMessage, "/utm/reference_path", latched_qos
        )
        self.reference_marker_pub = self.create_publisher(
            Marker, "/utm/reference_quality", latched_qos
        )
        self.rear_path_pub = self.create_publisher(
            PathMessage, "/utm/f9p/rear_path", visualization_qos
        )
        self.front_path_pub = self.create_publisher(
            PathMessage, "/utm/f9r/front_path", visualization_qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/utm/markers", visualization_qos
        )
        self.rear_point_pub = self.create_publisher(
            PointStamped, "/utm/f9p/rear", state_qos
        )
        self.front_point_pub = self.create_publisher(
            PointStamped, "/utm/f9r/front", state_qos
        )
        self.global_yaw_pub = self.create_publisher(Float64, "/global_yaw", state_qos)
        self.raw_yaw_pub = self.create_publisher(
            Float64, "/global_yaw/raw", state_qos
        )
        self.smoothed_yaw_pub = self.create_publisher(
            Float64, "/global_yaw/smoothed", state_qos
        )
        self.azimuth_pub = self.create_publisher(
            Float64, "/global_azimuth_deg", state_qos
        )
        self.yaw_variance_pub = self.create_publisher(
            Float64, "/global_yaw/variance", state_qos
        )
        self.valid_pub = self.create_publisher(Bool, "/global_yaw/valid", state_qos)
        self.heading_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/utm/global_yaw_pose", state_qos
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/utm/diagnostics", state_qos
        )
        self.rear_sub = self.create_subscription(
            NavSatFix, self.rear_topic, self.rear_callback, sensor_qos
        )
        self.front_sub = self.create_subscription(
            NavSatFix, self.front_topic, self.front_callback, sensor_qos
        )

    def _load_csv(self, csv_file):
        csv_path = Path(csv_file).expanduser().resolve()
        if not csv_path.is_file():
            raise RuntimeError(
                f"csv_file is required and must exist: {csv_path}. "
                "Run bag_to_enu_csv first."
            )
        rows = []
        with csv_path.open("r", newline="", encoding="utf-8") as input_file:
            reader = csv.DictReader(input_file)
            required = {
                "map_x",
                "map_y",
                "utm_zone",
                "hemisphere",
                "origin_easting",
                "origin_northing",
            }
            if not required.issubset(reader.fieldnames or []):
                missing = sorted(required.difference(reader.fieldnames or []))
                raise RuntimeError(f"CSV is missing required columns: {missing}")
            rows = list(reader)
        if len(rows) < 2:
            raise RuntimeError(f"CSV has fewer than two points: {csv_path}")

        first = rows[0]
        self.utm_zone = int(first["utm_zone"])
        self.northern = first["hemisphere"].upper() == "N"
        self.origin_easting = float(first["origin_easting"])
        self.origin_northing = float(first["origin_northing"])
        self.origin_altitude = float(first.get("origin_altitude", 0.0) or 0.0)

        path = PathMessage()
        path.header.frame_id = self.map_frame
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.ns = "utm_reference_quality"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.a = 1.0

        parsed = []
        for row in rows:
            x_value = float(row["map_x"])
            y_value = float(row["map_y"])
            z_value = float(row.get("map_z", 0.0) or 0.0)
            yaw = float(row.get("yaw", 0.0) or 0.0)
            segment = int(row.get("segment_id", 0) or 0)
            status = int(row.get("status", 0) or 0)
            parsed.append((x_value, y_value, z_value, yaw, segment, status))
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x_value
            pose.pose.position.y = y_value
            pose.pose.position.z = z_value
            quaternion = yaw_quaternion(yaw)
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)

        for first_point, second_point in zip(parsed, parsed[1:]):
            if first_point[4] != second_point[4]:
                continue
            marker.points.extend(
                [
                    Point(x=first_point[0], y=first_point[1], z=0.03),
                    Point(x=second_point[0], y=second_point[1], z=0.03),
                ]
            )
            fixed_pair = first_point[5] == 2 and second_point[5] == 2
            color = (
                ColorRGBA(r=0.12, g=0.85, b=0.30, a=1.0)
                if fixed_pair
                else ColorRGBA(r=0.95, g=0.58, b=0.12, a=0.9)
            )
            marker.colors.extend([color, color])
        self.get_logger().info(
            f"Loaded {len(path.poses)} CSV points from {csv_path}; "
            f"map origin=UTM {self.utm_zone}{'N' if self.northern else 'S'} "
            f"E={self.origin_easting:.3f} N={self.origin_northing:.3f}"
        )
        return path, marker

    def publish_reference(self):
        now = self.get_clock().now().to_msg()
        self.reference_path.header.stamp = now
        for pose in self.reference_path.poses:
            pose.header.stamp = now
        self.reference_marker.header.stamp = now
        self.reference_path_pub.publish(self.reference_path)
        self.reference_marker_pub.publish(self.reference_marker)

    def _sample_from_message(self, message):
        if (
            not math.isfinite(message.latitude)
            or not math.isfinite(message.longitude)
            or not -80.0 <= message.latitude <= 84.0
            or not -180.0 <= message.longitude <= 180.0
        ):
            return None
        try:
            easting, northing, _, northern = latlon_to_utm(
                message.latitude, message.longitude, zone=self.utm_zone
            )
        except ValueError:
            return None
        if northern != self.northern:
            return None
        stamp = stamp_seconds(message)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        horizontal_stddev = horizontal_standard_deviation(
            message.position_covariance, message.position_covariance_type
        )
        return FixSample(
            stamp=stamp,
            received_at=time.monotonic(),
            x=easting - self.origin_easting,
            y=northing - self.origin_northing,
            altitude=(
                message.altitude - self.origin_altitude
                if math.isfinite(message.altitude)
                else 0.0
            ),
            status=message.status.status,
            covariance_type=message.position_covariance_type,
            variance_east=message.position_covariance[0],
            variance_north=message.position_covariance[4],
            horizontal_stddev=horizontal_stddev,
            header_stamp=message.header.stamp,
        )

    def rear_callback(self, message):
        sample = self._sample_from_message(message)
        if sample is None:
            self.counts["rear_coordinate_rejected"] += 1
            return
        self._check_time_jump("rear", sample.stamp)
        self.counts["rear_received"] += 1
        self.rear_samples.append(sample)
        self._append_trail(self.rear_trail, sample)
        self.process_pending()

    def front_callback(self, message):
        sample = self._sample_from_message(message)
        if sample is None:
            self.counts["front_coordinate_rejected"] += 1
            return
        self._check_time_jump("front", sample.stamp)
        self.counts["front_received"] += 1
        self.front_samples.append(sample)
        self._append_trail(self.front_trail, sample)
        self.process_pending()

    def _check_time_jump(self, stream, stamp):
        attribute = f"last_{stream}_input_stamp"
        previous = getattr(self, attribute)
        if previous is not None and stamp < previous - 1.0:
            self.rear_samples.clear()
            self.front_samples.clear()
            self.rear_trail.clear()
            self.front_trail.clear()
            self.last_raw_yaw = None
            self.last_raw_stamp = None
            self.smoothed_yaw = None
            self.last_valid_wall = None
            self.last_rear_input_stamp = None
            self.last_front_input_stamp = None
            self.last_rejection = "input_time_jump"
            self.counts["time_jump_reset"] += 1
            if self.valid_state:
                self.valid_state = False
                self.valid_pub.publish(Bool(data=False))
            self.get_logger().warn(
                "Input timestamp moved backwards; cleared synchronizer and trails "
                "for a clean rosbag loop/restart."
            )
        setattr(self, attribute, stamp)

    def _append_trail(self, trail, sample):
        if not self.publish_trails or sample.status < 0:
            return
        if not trail or math.hypot(sample.x - trail[-1].x, sample.y - trail[-1].y) >= self.trail_min_distance:
            trail.append(sample)

    def _interpolated_rear(self, target_stamp):
        before = None
        after = None
        for sample in self.rear_samples:
            if sample.stamp <= target_stamp:
                before = sample
            if sample.stamp >= target_stamp:
                after = sample
                break
        if before is not None and after is not None:
            span = after.stamp - before.stamp
            if span == 0.0:
                return before, 0.0, "exact"
            if span <= self.max_interpolation_gap:
                fraction = (target_stamp - before.stamp) / span
                interpolated = FixSample(
                    stamp=target_stamp,
                    received_at=max(before.received_at, after.received_at),
                    x=before.x + fraction * (after.x - before.x),
                    y=before.y + fraction * (after.y - before.y),
                    altitude=before.altitude
                    + fraction * (after.altitude - before.altitude),
                    status=2 if before.fixed and after.fixed else min(before.status, after.status),
                    covariance_type=min(before.covariance_type, after.covariance_type),
                    variance_east=max(before.variance_east, after.variance_east),
                    variance_north=max(before.variance_north, after.variance_north),
                    horizontal_stddev=max(
                        before.horizontal_stddev, after.horizontal_stddev
                    ),
                    header_stamp=after.header_stamp,
                )
                sync_delta = max(
                    target_stamp - before.stamp, after.stamp - target_stamp
                )
                return interpolated, sync_delta, "interpolated"

        if not self.rear_samples:
            return None, math.inf, "missing"
        nearest = min(
            self.rear_samples, key=lambda sample: abs(sample.stamp - target_stamp)
        )
        delta = abs(nearest.stamp - target_stamp)
        if delta <= self.sync_tolerance:
            return nearest, delta, "nearest"
        return None, delta, "unsynchronized"

    def process_pending(self):
        now = time.monotonic()
        while self.front_samples and self.rear_samples:
            front = self.front_samples[0]
            latest_rear_stamp = self.rear_samples[-1].stamp
            if (
                latest_rear_stamp < front.stamp
                and now - front.received_at < self.pair_wait
            ):
                return
            rear, sync_delta, method = self._interpolated_rear(front.stamp)
            if rear is None and now - front.received_at < self.pair_wait:
                return
            self.front_samples.popleft()
            if rear is None:
                self._reject("sync")
                self.last_sync_delta = sync_delta
                continue
            self.counts[f"pair_{method}"] += 1
            self.last_sync_delta = sync_delta
            self.process_pair(rear, front)
            while (
                len(self.rear_samples) > 2
                and self.rear_samples[1].stamp < front.stamp - self.max_interpolation_gap
            ):
                self.rear_samples.popleft()

    def _fix_is_usable(self, sample):
        if sample.status < 0:
            return False, "no_fix"
        if self.require_rtk_fixed and not sample.fixed:
            return False, "not_fixed"
        if sample.horizontal_stddev > self.max_horizontal_stddev:
            return False, "covariance"
        return True, "ok"

    def process_pair(self, rear, front):
        rear_usable, rear_reason = self._fix_is_usable(rear)
        front_usable, front_reason = self._fix_is_usable(front)
        self.last_rear_stddev = rear.horizontal_stddev
        self.last_front_stddev = front.horizontal_stddev
        delta_x = front.x - rear.x
        delta_y = front.y - rear.y
        baseline = math.hypot(delta_x, delta_y)
        self.last_baseline = baseline
        if baseline > 1e-6:
            raw_yaw = math.atan2(delta_y, delta_x)
            self.raw_yaw_pub.publish(Float64(data=raw_yaw))
        else:
            raw_yaw = None

        reason = "ok"
        if not rear_usable:
            reason = f"rear_{rear_reason}"
        elif not front_usable:
            reason = f"front_{front_reason}"
        elif raw_yaw is None or not self.minimum_baseline <= baseline <= self.maximum_baseline:
            reason = "baseline"
        elif self.last_raw_yaw is not None and self.last_raw_stamp is not None:
            delta_time = front.stamp - self.last_raw_stamp
            if delta_time <= 0.0:
                reason = "timestamp"
            elif (
                abs(shortest_angular_distance(self.last_raw_yaw, raw_yaw) / delta_time)
                > self.max_yaw_rate
            ):
                reason = "yaw_rate"

        valid = reason == "ok"
        if not valid:
            self._reject(reason)
            self.publish_pair_markers(rear, front, raw_yaw, raw_yaw, reason)
            return

        self.last_raw_yaw = raw_yaw
        self.last_raw_stamp = front.stamp
        self.smoothed_yaw = circular_ema(
            self.smoothed_yaw, raw_yaw, self.smoothing_alpha
        )
        self.last_valid_wall = time.monotonic()
        self.last_rejection = "none"
        self.counts["accepted"] += 1
        if not self.valid_state:
            self.valid_state = True
            self.valid_pub.publish(Bool(data=True))

        yaw_variance = self._yaw_variance(rear, front, baseline)
        self.global_yaw_pub.publish(Float64(data=raw_yaw))
        self.smoothed_yaw_pub.publish(Float64(data=self.smoothed_yaw))
        self.azimuth_pub.publish(Float64(data=yaw_to_azimuth_degrees(raw_yaw)))
        self.yaw_variance_pub.publish(Float64(data=yaw_variance))
        self.rear_point_pub.publish(self._point_message(rear))
        self.front_point_pub.publish(self._point_message(front))
        self.heading_pose_pub.publish(
            self._heading_pose(rear, front, raw_yaw, yaw_variance)
        )
        self.publish_pair_markers(rear, front, raw_yaw, raw_yaw, "ok")

    def _reject(self, reason):
        self.counts[f"rejected_{reason}"] += 1
        self.last_rejection = reason

    @staticmethod
    def _yaw_variance(rear, front, baseline):
        transverse_variance = max(
            rear.variance_east,
            rear.variance_north,
            front.variance_east,
            front.variance_north,
            1e-6,
        )
        return min(math.pi**2, 2.0 * transverse_variance / max(baseline**2, 1e-6))

    def _point_message(self, sample):
        message = PointStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = sample.header_stamp
        message.point.x = sample.x
        message.point.y = sample.y
        message.point.z = sample.altitude
        return message

    def _heading_pose(self, rear, front, yaw, yaw_variance):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = front.header_stamp
        message.pose.pose.position.x = rear.x
        message.pose.pose.position.y = rear.y
        message.pose.pose.position.z = rear.altitude
        quaternion = yaw_quaternion(yaw)
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        covariance = message.pose.covariance
        covariance[0] = max(rear.variance_east, 1e-6)
        covariance[7] = max(rear.variance_north, 1e-6)
        covariance[14] = 9999.0
        covariance[21] = 9999.0
        covariance[28] = 9999.0
        covariance[35] = yaw_variance
        return message

    def _marker(self, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "dual_gnss_map"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    def publish_pair_markers(self, rear, front, raw_yaw, output_yaw, reason):
        stamp = front.header_stamp
        usable = reason == "ok"
        rear_marker = self._marker(0, Marker.SPHERE, stamp)
        rear_marker.pose.position.x = rear.x
        rear_marker.pose.position.y = rear.y
        rear_marker.scale.x = rear_marker.scale.y = rear_marker.scale.z = 0.28
        rear_marker.color.b = 1.0 if usable else 0.45
        rear_marker.color.r = 0.25 if usable else 0.55

        front_marker = self._marker(1, Marker.SPHERE, stamp)
        front_marker.pose.position.x = front.x
        front_marker.pose.position.y = front.y
        front_marker.scale.x = front_marker.scale.y = front_marker.scale.z = 0.28
        front_marker.color.r = 1.0 if usable else 0.55
        front_marker.color.b = 0.2 if usable else 0.45

        baseline_marker = self._marker(2, Marker.LINE_STRIP, stamp)
        baseline_marker.scale.x = 0.07
        baseline_marker.color.r = baseline_marker.color.g = baseline_marker.color.b = 0.60
        baseline_marker.points = [
            Point(x=rear.x, y=rear.y, z=0.12),
            Point(x=front.x, y=front.y, z=0.12),
        ]

        heading_marker = self._marker(3, Marker.ARROW, stamp)
        if output_yaw is None:
            heading_marker.action = Marker.DELETE
        else:
            heading_marker.scale.x = 0.10
            heading_marker.scale.y = 0.22
            heading_marker.scale.z = 0.22
            if usable:
                heading_marker.color.r = 1.0
                heading_marker.color.g = 0.88
                heading_marker.color.b = 0.05
            else:
                heading_marker.color.r = 0.65
                heading_marker.color.g = 0.65
                heading_marker.color.b = 0.65
            heading_marker.points = [
                Point(x=rear.x, y=rear.y, z=0.24),
                Point(
                    x=rear.x + 1.8 * math.cos(output_yaw),
                    y=rear.y + 1.8 * math.sin(output_yaw),
                    z=0.24,
                ),
            ]

        text_marker = self._marker(4, Marker.TEXT_VIEW_FACING, stamp)
        text_marker.pose.position.x = 0.5 * (rear.x + front.x)
        text_marker.pose.position.y = 0.5 * (rear.y + front.y) - 1.8
        text_marker.pose.position.z = 0.6
        text_marker.scale.z = 0.42
        if usable:
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
        else:
            text_marker.color.r = 1.0
            text_marker.color.g = 0.35
            text_marker.color.b = 0.15
        if output_yaw is None:
            yaw_text = "Global yaw: waiting for first valid pair"
            output_text = f"INVALID ({reason})"
        elif usable:
            yaw_text = f"Global yaw: {math.degrees(output_yaw):.1f} deg"
            output_text = "VALID"
        else:
            yaw_text = f"Candidate yaw: {math.degrees(output_yaw):.1f} deg"
            output_text = f"INVALID ({reason})"
        text_marker.text = (
            f"{yaw_text}\n"
            f"Baseline: {self.last_baseline:.2f} m\n"
            f"F9P rear: {'FIXED' if rear.fixed else 'NON-FIXED'}\n"
            f"F9R front: {'FIXED' if front.fixed else 'NON-FIXED'}\n"
            f"Output: {output_text}"
        )
        self.marker_pub.publish(
            MarkerArray(
                markers=[
                    rear_marker,
                    front_marker,
                    baseline_marker,
                    heading_marker,
                    text_marker,
                ]
            )
        )

    def _path_message(self, trail):
        message = PathMessage()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for sample in trail:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = sample.header_stamp
            pose.pose.position.x = sample.x
            pose.pose.position.y = sample.y
            pose.pose.position.z = sample.altitude
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def publish_trails_callback(self):
        if not self.publish_trails:
            return
        self.rear_path_pub.publish(self._path_message(self.rear_trail))
        self.front_path_pub.publish(self._path_message(self.front_trail))

    def publish_status(self):
        now = time.monotonic()
        current_valid = (
            self.last_valid_wall is not None
            and now - self.last_valid_wall <= self.heading_timeout
        )
        if current_valid != self.valid_state:
            self.valid_state = current_valid
            self.valid_pub.publish(Bool(data=current_valid))
            if not current_valid:
                self.last_rejection = "timeout"
                self.smoothed_yaw = None
                self.last_raw_yaw = None
                self.last_raw_stamp = None

        diagnostic = DiagnosticStatus()
        diagnostic.name = "utm_pkg/dual_gnss_global_yaw"
        diagnostic.hardware_id = "f9p_rear+f9r_front"
        diagnostic.level = (
            DiagnosticStatus.OK
            if current_valid
            else DiagnosticStatus.WARN
        )
        diagnostic.message = "global yaw valid" if current_valid else "heading_timeout"
        values = {
            "valid": current_valid,
            "require_rtk_fixed": self.require_rtk_fixed,
            "baseline_m": self.last_baseline,
            "sync_delta_sec": self.last_sync_delta,
            "rear_horizontal_stddev_m": self.last_rear_stddev,
            "front_horizontal_stddev_m": self.last_front_stddev,
            "rear_received": self.counts["rear_received"],
            "front_received": self.counts["front_received"],
            "accepted": self.counts["accepted"],
            "last_rejection": self.last_rejection,
        }
        for key in sorted(self.counts):
            if key.startswith("rejected_") or key.startswith("pair_"):
                values[key] = self.counts[key]
        diagnostic.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualGnssMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
