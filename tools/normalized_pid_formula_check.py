#!/usr/bin/env python3
"""No-hardware checks for the Nano normalized PID tilt mapping."""

POSITION_LIMIT_X_MM = 260.0
POSITION_LIMIT_Y_MM = 200.0
MAX_PLATFORM_TILT_X_DEG = 8.0
MAX_PLATFORM_TILT_Y_DEG = 8.0
KP = 1.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalized_error(target_mm: float, ball_mm: float, limit_mm: float) -> float:
    return clamp((target_mm - ball_mm) / limit_mm, -1.0, 1.0)


def p_only_output(error_norm: float, kp: float = KP) -> float:
    return clamp(kp * error_norm, -1.0, 1.0)


def requested_tilt(output_norm: float, max_tilt_deg: float) -> float:
    return output_norm * max_tilt_deg


def test_full_scale_edges() -> None:
    ex = normalized_error(0.0, -POSITION_LIMIT_X_MM, POSITION_LIMIT_X_MM)
    ux = p_only_output(ex)
    assert ex == 1.0
    assert ux == 1.0
    assert requested_tilt(ux, MAX_PLATFORM_TILT_X_DEG) == 8.0

    ey = normalized_error(0.0, POSITION_LIMIT_Y_MM, POSITION_LIMIT_Y_MM)
    uy = p_only_output(ey)
    assert ey == -1.0
    assert uy == -1.0
    assert requested_tilt(uy, MAX_PLATFORM_TILT_Y_DEG) == -8.0


def test_half_gain_is_half_tilt() -> None:
    ex = normalized_error(0.0, -POSITION_LIMIT_X_MM, POSITION_LIMIT_X_MM)
    ux = p_only_output(ex, kp=0.5)
    assert ux == 0.5
    assert requested_tilt(ux, MAX_PLATFORM_TILT_X_DEG) == 4.0


def test_out_of_range_clamps_before_tilt() -> None:
    ex = normalized_error(0.0, -999.0, POSITION_LIMIT_X_MM)
    ux = p_only_output(ex)
    assert ex == 1.0
    assert ux == 1.0
    assert requested_tilt(ux, MAX_PLATFORM_TILT_X_DEG) == MAX_PLATFORM_TILT_X_DEG


if __name__ == "__main__":
    test_full_scale_edges()
    test_half_gain_is_half_tilt()
    test_out_of_range_clamps_before_tilt()
    print("normalized PID mapping checks passed")
