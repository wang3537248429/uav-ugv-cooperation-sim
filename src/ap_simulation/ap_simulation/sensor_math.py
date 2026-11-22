"""Coordinate and range calculations for the ideal optical-flow sensor.

No ROS runtime is required; pose inputs only need position/orientation fields.
"""

import math


def quaternion_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def ned_velocity_to_body_flu(velocity_ned, roll, pitch, yaw):
    """Rotate NED velocity into ROS body FLU coordinates."""
    north, east, down = velocity_ned
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R maps body FRD into NED; its transpose maps NED into body FRD.
    body_forward = cp * cy * north + cp * sy * east - sp * down
    body_right = (
        (sr * sp * cy - cr * sy) * north
        + (sr * sp * sy + cr * cy) * east
        + sr * cp * down
    )
    body_down = (
        (cr * sp * cy + sr * sy) * north
        + (cr * sp * sy - sr * cy) * east
        + cr * cp * down
    )
    return body_forward, -body_right, -body_down


def slant_range_to_ground(height_m, roll, pitch):
    """Return range along a body-down sensor ray over a flat ground plane."""
    vertical_projection = math.cos(roll) * math.cos(pitch)
    if not math.isfinite(height_m) or height_m <= 0.0 or vertical_projection <= 0.1:
        return None
    return height_m / vertical_projection


def pose_displacement_to_body_velocity(
    previous, current, max_dt, max_speed, attitude_rpy=None
):
    """Convert one truth-pose interval into a horizontal body-FLU velocity.

    ``previous`` and ``current`` are ``(stamp_seconds, PoseStamped)`` pairs.
    The interval is deliberately evaluated by the publishing loop instead of
    by the truth callback.  Gazebo can deliver truth at a rate that is not an
    integer multiple of the optical-flow rate; differentiating every truth
    callback and holding the last derivative would then alias acceleration
    into a biased velocity.  Averaging the displacement over the complete
    flow sample interval preserves the displacement seen by the sensor. When
    supplied, attitude_rpy aligns the synthetic measurement with the attitude
    estimate used by the odometry integrator.
    """
    if previous is None or current is None:
        return None
    previous_stamp, previous_msg = previous
    current_stamp, current_msg = current
    dt = float(current_stamp) - float(previous_stamp)
    if not 0.001 <= dt <= float(max_dt):
        return None

    displacement = (
        float(current_msg.pose.position.x) - float(previous_msg.pose.position.x),
        float(current_msg.pose.position.y) - float(previous_msg.pose.position.y),
        float(current_msg.pose.position.z) - float(previous_msg.pose.position.z),
    )
    velocity_ned = tuple(value / dt for value in displacement)
    speed = math.sqrt(sum(value * value for value in velocity_ned))
    if attitude_rpy is None:
        q = current_msg.pose.orientation
        attitude = quaternion_to_euler(q.x, q.y, q.z, q.w)
    else:
        attitude = tuple(float(value) for value in attitude_rpy)
    if speed > float(max_speed) or not all(
        math.isfinite(value) for value in (*velocity_ned, *attitude)
    ):
        return None

    velocity_flu = ned_velocity_to_body_flu(velocity_ned, *attitude)
    # A down-looking optical-flow camera cannot observe vertical translation;
    # the range sensor supplies that constraint.
    return velocity_flu[0], velocity_flu[1], 0.0
