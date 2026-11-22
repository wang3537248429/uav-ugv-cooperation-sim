"""ROS message integration for the synthetic optical-flow sensor."""

import math

from geometry_msgs.msg import PoseStamped

from ap_simulation.optical_flow_odometry import OpticalFlowIntegrator, OpticalFlowSample
from ap_simulation.sim_optical_flow_sensor import pose_displacement_to_body_velocity


def test_flow_sensor_uses_full_pose_interval_for_average_velocity():
    previous = PoseStamped()
    current = PoseStamped()
    current.pose.position.x = 0.30
    current.pose.position.y = -0.15
    current.pose.position.z = 0.20
    current.pose.orientation.w = 1.0

    velocity = pose_displacement_to_body_velocity(
        (12.0, previous), (12.15, current), max_dt=0.20, max_speed=3.0
    )

    assert velocity is not None
    assert math.isclose(velocity[0], 2.0, abs_tol=1e-9)
    assert math.isclose(velocity[1], 1.0, abs_tol=1e-9)
    # A downward optical-flow camera does not measure vertical translation.
    assert velocity[2] == 0.0


def test_sim_flow_and_odometry_share_the_truth_attitude_reference():
    previous = PoseStamped()
    previous.pose.orientation.w = 1.0
    current = PoseStamped()
    current.pose.position.y = 0.1
    # The flow sensor and estimator use the same truth attitude reference.
    current.pose.orientation.z = math.sin(-0.5 / 2.0)
    current.pose.orientation.w = math.cos(-0.5 / 2.0)
    truth_attitude = (0.0, 0.0, -0.5)

    velocity_body = pose_displacement_to_body_velocity(
        (1.0, previous),
        (1.1, current),
        max_dt=0.2,
        max_speed=3.0,
        attitude_rpy=truth_attitude,
    )
    assert velocity_body is not None
    estimator = OpticalFlowIntegrator(max_dt=0.2)
    estimator.update(OpticalFlowSample(1.0, (0.0, 0.0, 0.0), 255, 1.0))
    result = estimator.update(
        OpticalFlowSample(1.1, velocity_body, 255, 1.0), truth_attitude
    )

    assert result is not None
    assert math.isclose(result.position_ned[0], 0.0, abs_tol=1e-9)
    assert math.isclose(result.position_ned[1], 0.1, abs_tol=1e-9)
