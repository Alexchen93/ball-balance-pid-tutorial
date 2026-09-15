#!/usr/bin/env python3
"""No-hardware checks for the Nano asymmetric edge-to-endpoint PID mapping."""

SERVO_X_CENTER = 74
SERVO_Y_CENTER = 76
SERVO_LIMIT_OFFSET_DEG = 20
SERVO_X_MIN_ANGLE = SERVO_X_CENTER - SERVO_LIMIT_OFFSET_DEG
SERVO_X_MAX_ANGLE = SERVO_X_CENTER + SERVO_LIMIT_OFFSET_DEG
SERVO_Y_MIN_ANGLE = SERVO_Y_CENTER - SERVO_LIMIT_OFFSET_DEG
SERVO_Y_MAX_ANGLE = SERVO_Y_CENTER + SERVO_LIMIT_OFFSET_DEG
MAX_PLATFORM_TILT_X_DEG = float(SERVO_LIMIT_OFFSET_DEG)
MAX_PLATFORM_TILT_Y_DEG = float(SERVO_LIMIT_OFFSET_DEG)
KP = 1.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalized_error(target_mm: float, ball_mm: float, min_mm: float, max_mm: float) -> float:
    error_mm = target_mm - ball_mm
    denominator = target_mm - min_mm if error_mm >= 0.0 else max_mm - target_mm
    if denominator <= 0.0:
        return 0.0
    return clamp(error_mm / denominator, -1.0, 1.0)


def p_only_output(error_norm: float, kp: float = KP) -> float:
    return clamp(kp * error_norm, -1.0, 1.0)


def requested_tilt(output_norm: float, max_tilt_deg: float) -> float:
    return output_norm * max_tilt_deg


def servo_angle(center: int, direction: int, requested_tilt_deg: float, low: int, high: int) -> int:
    return round(clamp(center + direction * requested_tilt_deg, low, high))


def test_asymmetric_edges_are_independent_full_scale() -> None:
    x_min, x_max = -290.0, 230.0
    y_min, y_max = -180.0, 220.0

    assert normalized_error(0.0, x_min, x_min, x_max) == 1.0
    assert normalized_error(0.0, x_max, x_min, x_max) == -1.0
    assert normalized_error(0.0, y_min, y_min, y_max) == 1.0
    assert normalized_error(0.0, y_max, y_min, y_max) == -1.0


def test_safe_endpoint_offset_is_twenty_degrees_per_axis() -> None:
    x_output = p_only_output(normalized_error(0.0, -290.0, -290.0, 230.0))
    y_output = p_only_output(normalized_error(0.0, 220.0, -180.0, 220.0))

    assert requested_tilt(x_output, MAX_PLATFORM_TILT_X_DEG) == 20.0
    assert requested_tilt(y_output, MAX_PLATFORM_TILT_Y_DEG) == -20.0
    assert servo_angle(SERVO_X_CENTER, -1, 20.0, SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE) == SERVO_X_MIN_ANGLE
    assert servo_angle(SERVO_Y_CENTER, 1, -20.0, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE) == SERVO_Y_MIN_ANGLE


def test_half_gain_is_half_safe_endpoint_offset() -> None:
    ex = normalized_error(0.0, -290.0, -290.0, 230.0)
    ux = p_only_output(ex, kp=0.5)

    assert ux == 0.5
    assert requested_tilt(ux, MAX_PLATFORM_TILT_X_DEG) == 10.0


def test_out_of_range_clamps_before_tilt() -> None:
    ex = normalized_error(0.0, -999.0, -290.0, 230.0)
    ux = p_only_output(ex)

    assert ex == 1.0
    assert ux == 1.0
    assert requested_tilt(ux, MAX_PLATFORM_TILT_X_DEG) == MAX_PLATFORM_TILT_X_DEG


if __name__ == "__main__":
    test_asymmetric_edges_are_independent_full_scale()
    test_safe_endpoint_offset_is_twenty_degrees_per_axis()
    test_half_gain_is_half_safe_endpoint_offset()
    test_out_of_range_clamps_before_tilt()
    print("asymmetric PID edge mapping checks passed")
