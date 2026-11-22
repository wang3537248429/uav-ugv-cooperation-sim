"""Unit tests for the hardware-facing 3-D navigation primitives."""

import math

from ap_mission.flight_control_3d import (
    GuidedController3D,
    project_to_segment,
    staged_return_step,
    staged_return_target,
)
from ap_simulation.optical_flow_odometry import (
    FlowTimestampMapper,
    OpticalFlowIntegrator,
    OpticalFlowSample,
    body_flu_to_ned,
    ned_to_body_flu,
    optical_flow_body_to_ned,
    attitude_rpy_to_ned_flu_quaternion,
)
from ap_simulation.sensor_math import (
    ned_velocity_to_body_flu,
    slant_range_to_ground,
)
from ap_simulation.timebase import SensorDrivenClock, SourceClockMapper


def test_sitl_clock_does_not_run_ahead_when_physics_repeats_samples():
    clock = SensorDrivenClock()
    first = clock.advance(1_000_000_000)
    for _ in range(100):
        assert clock.advance(1_000_000_000) is None
    later = clock.advance(1_050_000_000)
    assert math.isclose(later - first, .05, abs_tol=1e-9)
    assert clock.advance(1_040_000_000) is None


def test_segment_projection_and_lookahead_are_three_dimensional():
    projection = project_to_segment((0.5, 0.2, 0.5), (0.0, 0.0, 0.0), (1.0, 0.0, 1.0))
    assert math.isclose(projection.point[0], 0.5, abs_tol=1e-6)
    assert math.isclose(projection.point[2], 0.5, abs_tol=1e-6)
    assert math.isclose(projection.cross_track[1], 0.2, abs_tol=1e-6)


def test_flow_integrator_rejects_bad_quality_and_tracks_range_height():
    estimator = OpticalFlowIntegrator(min_quality=100, max_dt=0.2)
    assert estimator.update(OpticalFlowSample(1.0, (1.0, 0.0, 0.0), 20, 1.0)) is None
    assert estimator.update(OpticalFlowSample(1.0, (0.0, 0.0, 0.0), 200, 1.2)) is not None
    result = estimator.update(OpticalFlowSample(1.1, (1.0, 0.0, 0.0), 200, 1.1))
    assert result is not None
    assert result.position_ned[0] > 0.0
    assert math.isclose(result.position_ned[2], 0.1, abs_tol=1e-6)


def test_flow_body_flu_axis_conversion_matches_ned_contract():
    north, east, down = body_flu_to_ned((1.0, 0.0, 0.0), 0.0, 0.0, 0.0)
    assert (north, east, down) == (1.0, 0.0, 0.0)
    north, east, down = body_flu_to_ned((0.0, 1.0, 0.0), 0.0, 0.0, 0.0)
    assert (north, east, down) == (0.0, -1.0, 0.0)


def test_ned_and_body_flu_velocity_transforms_are_inverse_under_attitude():
    attitude = (0.31, -0.27, 1.2)
    velocity_ned = (1.2, -0.8, 0.4)
    velocity_body = ned_to_body_flu(velocity_ned, *attitude)

    assert all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(
            body_flu_to_ned(velocity_body, *attitude), velocity_ned
        )
    )


def test_odometry_orientation_quaternion_uses_ros_flu_body_axes():
    attitude = (0.2, -0.3, 0.7)
    quaternion = attitude_rpy_to_ned_flu_quaternion(*attitude)
    vector_body = (0.4, -0.6, 0.8)
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * vector_body[2] - qz * vector_body[1])
    ty = 2.0 * (qz * vector_body[0] - qx * vector_body[2])
    tz = 2.0 * (qx * vector_body[1] - qy * vector_body[0])
    rotated = (
        vector_body[0] + qw * tx + qy * tz - qz * ty,
        vector_body[1] + qw * ty + qz * tx - qx * tz,
        vector_body[2] + qw * tz + qx * ty - qy * tx,
    )
    expected = body_flu_to_ned(vector_body, *attitude)

    assert all(
        math.isclose(actual, expected_value, abs_tol=1e-9)
        for actual, expected_value in zip(rotated, expected)
    )


def test_sim_sensor_rotates_ned_truth_velocity_into_body_flu():
    assert ned_velocity_to_body_flu((1.0, 0.0, 0.0), 0.0, 0.0, 0.0) == (
        1.0, 0.0, 0.0
    )
    forward_at_east_heading = ned_velocity_to_body_flu(
        (0.0, 1.0, 0.0), 0.0, 0.0, math.pi / 2.0
    )
    assert math.isclose(forward_at_east_heading[0], 1.0, abs_tol=1e-9)
    assert math.isclose(forward_at_east_heading[1], 0.0, abs_tol=1e-9)


def test_sim_sensor_range_accounts_for_downward_mount_tilt():
    assert math.isclose(slant_range_to_ground(2.0, 0.0, 0.0), 2.0)
    assert math.isclose(slant_range_to_ground(2.0, math.pi / 3.0, 0.0), 4.0)
    assert slant_range_to_ground(0.0, 0.0, 0.0) is None


def test_range_and_planar_flow_recover_three_dimensional_ned_velocity():
    attitude = (0.23, -0.31, 1.07)
    for expected in ((1.2, -0.4, 0.25), (0.0, 0.0, 0.6)):
        flow = ned_velocity_to_body_flu(expected, *attitude)
        actual = optical_flow_body_to_ned(
            (flow[0], flow[1], 0.0), expected[2], *attitude
        )
        assert all(
            math.isclose(got, want, abs_tol=1e-9)
            for got, want in zip(actual, expected)
        )


def test_bad_flow_velocity_consumes_its_time_and_range_baseline():
    estimator = OpticalFlowIntegrator(min_quality=80, max_dt=0.2, max_speed=3.0)
    assert estimator.update(OpticalFlowSample(1.0, (0.0, 0.0, 0.0), 200, 1.0))
    assert estimator.update(OpticalFlowSample(1.1, (8.0, 0.0, 0.0), 200, 1.0)) is None

    result = estimator.update(OpticalFlowSample(1.2, (0.5, 0.0, 0.0), 200, 1.0))

    assert result is not None
    assert math.isclose(result.position_ned[0], 0.05, abs_tol=1e-9)


def test_filtered_velocity_does_not_bias_interval_position_integration():
    estimator = OpticalFlowIntegrator(
        min_quality=80, max_dt=0.2, max_speed=3.0, velocity_alpha=0.2
    )
    assert estimator.update(OpticalFlowSample(1.0, (0.0, 0.0, 0.0), 200, 1.0))

    result = estimator.update(OpticalFlowSample(1.1, (1.0, 0.0, 0.0), 200, 1.0))

    assert result is not None
    assert math.isclose(result.velocity_ned[0], 0.2, abs_tol=1e-9)
    assert math.isclose(result.position_ned[0], 0.1, abs_tol=1e-9)


def test_gazebo_source_clock_is_mapped_without_losing_sample_spacing():
    mapper = SourceClockMapper()
    first = mapper.to_ros_nanoseconds((0, 0), 10_000_000_000)
    second = mapper.to_ros_nanoseconds((0, 20_000_000), 10_100_000_000)

    assert first == 10_000_000_000
    assert second - first == 20_000_000

    restarted = mapper.to_ros_nanoseconds((0, 0), 11_000_000_000)
    assert restarted == 11_000_000_000


def test_flow_timestamps_never_switch_clock_domains_at_zero_epoch():
    mapper = FlowTimestampMapper()
    first = mapper.to_monotonic_seconds(0.0, 100.0)
    second = mapper.to_monotonic_seconds(0.02, 100.02)
    restarted = mapper.to_monotonic_seconds(0.01, 100.04)

    assert math.isclose(second - first, 0.02, abs_tol=1e-9)
    assert math.isclose(restarted, 100.04, abs_tol=1e-9)


def test_guided_controller_limits_full_3d_command():
    command = GuidedController3D().velocity_command((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (5.0, 5.0, 5.0))
    assert math.hypot(command[0], command[1]) <= 0.30
    assert abs(command[2]) <= 0.15


def test_post_strike_return_uses_an_axis_aligned_safe_corridor():
    anchor = (0.13, 1.83)

    corridor_target = staged_return_target(anchor, "RETURN_Y")
    home_target = staged_return_target(anchor, "RETURN_X")

    assert corridor_target == (anchor[0], 0.0)
    assert home_target == (0.0, 0.0)

    first_leg_step = staged_return_step(
        (0.11, 1.82), anchor, "RETURN_Y", 0.20
    )
    final_leg_step = staged_return_step(
        (0.13, 0.10), anchor, "RETURN_X", 0.20
    )
    assert first_leg_step == (anchor[0], 1.62)
    assert final_leg_step == (0.0, 0.0)


def test_post_strike_return_rejects_unknown_phase():
    try:
        staged_return_target((0.0, 1.0), "RETURN")
    except ValueError as error:
        assert "unsupported" in str(error)
    else:
        raise AssertionError("unknown return phases must be rejected")
