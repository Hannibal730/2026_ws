"""Pure helpers shared by live processing and tests."""

import math

from .geodesy import normalize_angle, shortest_angular_distance


def horizontal_standard_deviation(covariance, covariance_type):
    """Return conservative horizontal 1-sigma uncertainty, or infinity."""
    if covariance_type == 0 or len(covariance) < 5:
        return math.inf
    east_variance = covariance[0]
    north_variance = covariance[4]
    if not math.isfinite(east_variance) or not math.isfinite(north_variance):
        return math.inf
    if east_variance < 0.0 or north_variance < 0.0:
        return math.inf
    return math.sqrt(max(east_variance, north_variance))


def circular_ema(previous_yaw, current_yaw, alpha):
    if previous_yaw is None:
        return normalize_angle(current_yaw)
    alpha = min(1.0, max(0.0, alpha))
    x_value = (1.0 - alpha) * math.cos(previous_yaw) + alpha * math.cos(current_yaw)
    y_value = (1.0 - alpha) * math.sin(previous_yaw) + alpha * math.sin(current_yaw)
    if math.hypot(x_value, y_value) < 1e-12:
        return normalize_angle(current_yaw)
    return math.atan2(y_value, x_value)


def yaw_rate(previous_yaw, current_yaw, delta_time):
    if previous_yaw is None or delta_time <= 0.0:
        return 0.0
    return shortest_angular_distance(previous_yaw, current_yaw) / delta_time

