"""Conversions between body-FLU velocity and down-facing optical flow."""

import math
from typing import Optional, Tuple


def velocity_to_flow_rates(
    velocity_forward_mps: float,
    velocity_left_mps: float,
    height_m: float,
    *,
    sensor_yaw_rad: float = 0.0,
    x_sign: float = 1.0,
    y_sign: float = 1.0,
    max_rate_rad_s: float = 8.0,
) -> Optional[Tuple[float, float]]:
    """Return sensor-axis angular flow rates for a down-facing sensor.

    The input velocity uses ROS body FLU. MAVLink optical-flow axes use a
    forward-right-down sensor frame: positive sensor Y motion produces
    negative X flow, while positive sensor X motion produces positive Y flow.
    """
    values = (
        velocity_forward_mps,
        velocity_left_mps,
        height_m,
        sensor_yaw_rad,
        x_sign,
        y_sign,
        max_rate_rad_s,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    if height_m <= 0.0 or max_rate_rad_s <= 0.0:
        return None

    cosine = math.cos(sensor_yaw_rad)
    sine = math.sin(sensor_yaw_rad)
    velocity_right_mps = -velocity_left_mps
    velocity_sensor_x = cosine * velocity_forward_mps + sine * velocity_right_mps
    velocity_sensor_y = -sine * velocity_forward_mps + cosine * velocity_right_mps
    flow_x = -x_sign * velocity_sensor_y / height_m
    flow_y = y_sign * velocity_sensor_x / height_m
    if max(abs(flow_x), abs(flow_y)) > max_rate_rad_s:
        return None
    return flow_x, flow_y
