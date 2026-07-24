import math

from utm_pkg.quality import (
    circular_ema,
    horizontal_standard_deviation,
)


def test_unknown_covariance_is_rejected():
    assert math.isinf(horizontal_standard_deviation([0.0] * 9, 0))


def test_horizontal_standard_deviation_uses_worst_axis():
    covariance = [0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 1.0]
    assert math.isclose(horizontal_standard_deviation(covariance, 2), 0.3)


def test_circular_filter_crosses_wrap_without_flipping():
    result = circular_ema(math.radians(179.0), math.radians(-179.0), 0.5)
    assert abs(abs(math.degrees(result)) - 180.0) < 0.1
