"""Small dependency-free WGS84/UTM helpers.

The local map coordinates used by this package are:
  map_x = UTM easting - origin easting
  map_y = UTM northing - origin northing

For a competition-sized site this is the practical local ENU representation:
x points east, y points north, and z points up.
"""

import math


WGS84_A = 6378137.0
WGS84_E_SQ = 0.0066943799901413165
UTM_SCALE = 0.9996


def utm_zone_for_longitude(longitude):
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude must be between -180 and 180 degrees")
    return min(60, max(1, int((longitude + 180.0) / 6.0) + 1))


def latlon_to_utm(latitude, longitude, zone=None):
    """Convert a WGS84 coordinate to UTM easting/northing."""
    if not -80.0 <= latitude <= 84.0:
        raise ValueError("UTM latitude must be between -80 and 84 degrees")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude must be between -180 and 180 degrees")

    if zone is None:
        zone = utm_zone_for_longitude(longitude)
    if not 1 <= int(zone) <= 60:
        raise ValueError("UTM zone must be between 1 and 60")
    zone = int(zone)

    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)
    central_meridian = math.radians(zone * 6.0 - 183.0)
    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    tan_latitude = math.tan(latitude_rad)

    eccentricity_prime_sq = WGS84_E_SQ / (1.0 - WGS84_E_SQ)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E_SQ * sin_latitude**2)
    tangent_sq = tan_latitude**2
    second_eccentricity = eccentricity_prime_sq * cos_latitude**2
    longitude_term = cos_latitude * (longitude_rad - central_meridian)

    e2 = WGS84_E_SQ
    e4 = e2**2
    e6 = e2**3
    meridional_arc = WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0)
        * latitude_rad
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
        * math.sin(2.0 * latitude_rad)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0)
        * math.sin(4.0 * latitude_rad)
        - 35.0 * e6 / 3072.0 * math.sin(6.0 * latitude_rad)
    )

    easting = 500000.0 + UTM_SCALE * radius * (
        longitude_term
        + (1.0 - tangent_sq + second_eccentricity) * longitude_term**3 / 6.0
        + (
            5.0
            - 18.0 * tangent_sq
            + tangent_sq**2
            + 72.0 * second_eccentricity
            - 58.0 * eccentricity_prime_sq
        )
        * longitude_term**5
        / 120.0
    )
    northing = UTM_SCALE * (
        meridional_arc
        + radius
        * tan_latitude
        * (
            longitude_term**2 / 2.0
            + (
                5.0
                - tangent_sq
                + 9.0 * second_eccentricity
                + 4.0 * second_eccentricity**2
            )
            * longitude_term**4
            / 24.0
            + (
                61.0
                - 58.0 * tangent_sq
                + tangent_sq**2
                + 600.0 * second_eccentricity
                - 330.0 * eccentricity_prime_sq
            )
            * longitude_term**6
            / 720.0
        )
    )
    northern = latitude >= 0.0
    if not northern:
        northing += 10000000.0
    return easting, northing, zone, northern


def normalize_angle(angle):
    """Normalize radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angular_distance(start, end):
    return normalize_angle(end - start)


def yaw_to_azimuth_degrees(yaw):
    """ROS ENU yaw (east=0, CCW) to compass azimuth (north=0, clockwise)."""
    return (90.0 - math.degrees(yaw)) % 360.0


def yaw_quaternion(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)

