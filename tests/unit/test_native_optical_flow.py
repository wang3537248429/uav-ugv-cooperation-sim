"""Regression tests for the native ArduPilot optical-flow bridge contract."""

import math


from ap_mavlink_bridge.optical_flow import velocity_to_flow_rates


def test_body_flu_velocity_maps_to_down_sensor_flow_axes():
    forward_flow = velocity_to_flow_rates(1.0, 0.0, 2.0)
    left_flow = velocity_to_flow_rates(0.0, 1.0, 2.0)
    assert forward_flow == (0.0, 0.5)
    assert left_flow == (0.5, 0.0)


def test_flow_axis_signs_are_calibratable_and_rates_are_bounded():
    assert velocity_to_flow_rates(
        1.0, 0.5, 1.0, x_sign=-1.0, y_sign=-1.0
    ) == (-0.5, -1.0)
    assert velocity_to_flow_rates(10.0, 0.0, 1.0, max_rate_rad_s=8.0) is None
    assert velocity_to_flow_rates(0.0, 0.0, 0.0) is None
    assert velocity_to_flow_rates(math.nan, 0.0, 1.0) is None


def test_sensor_yaw_rotates_velocity_into_sensor_axes():
    flow = velocity_to_flow_rates(
        1.0, 0.0, 1.0, sensor_yaw_rad=math.pi / 2.0
    )
    assert flow is not None
    assert math.isclose(flow[0], 1.0, abs_tol=1.0e-9)
    assert math.isclose(flow[1], 0.0, abs_tol=1.0e-9)
