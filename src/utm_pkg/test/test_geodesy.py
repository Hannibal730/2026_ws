import math

from utm_pkg.geodesy import (
    latlon_to_utm,
    normalize_angle,
    shortest_angular_distance,
    yaw_to_azimuth_degrees,
)


def test_known_seoul_utm_coordinate():
    # PROJ reference for 37.5665 N, 126.9780 E in WGS84 / UTM zone 52N.
    easting, northing, zone, northern = latlon_to_utm(37.5665, 126.9780)
    assert zone == 52
    assert northern
    assert abs(easting - 321424.29) < 0.5
    assert abs(northing - 4159640.64) < 0.5


def test_ros_yaw_to_compass_azimuth():
    assert math.isclose(yaw_to_azimuth_degrees(0.0), 90.0)
    assert math.isclose(yaw_to_azimuth_degrees(math.pi / 2.0), 0.0)
    assert math.isclose(yaw_to_azimuth_degrees(math.pi), 270.0)


def test_angle_wrap():
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert math.isclose(
        shortest_angular_distance(math.radians(179.0), math.radians(-179.0)),
        math.radians(2.0),
    )

