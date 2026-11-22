"""Publish idealized optical-flow and range samples from Gazebo truth.

This sensor model exercises the ROS/MAVLink integration path. It derives
translation-only velocity and down-facing range from Gazebo truth, expressing
flow in the same truth attitude provided to the odometry node. It also
publishes that attitude as an idealized simulation input. It is not a camera
image simulator and does not model visual texture or blur.
"""

import time

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8

from .sensor_math import (
    pose_displacement_to_body_velocity,
    quaternion_to_euler,
    slant_range_to_ground,
)
from .sensor_math import ned_velocity_to_body_flu  # noqa: F401 (compatibility export)


class SimOpticalFlowSensor(Node):
    """Truth-derived optical-flow and downward-range source for SITL tests."""

    def __init__(self):
        super().__init__("sim_optical_flow_sensor")
        self.declare_parameter("truth_topic", "/sim/drone_pose_ned")
        self.declare_parameter("truth_attitude_topic", "/sim/truth_attitude")
        self.declare_parameter("flow_topic", "/optical_flow/velocity")
        self.declare_parameter("quality_topic", "/optical_flow/quality")
        self.declare_parameter("range_topic", "/range/down")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("initial_height_m", 0.24)
        self.declare_parameter("minimum_range_m", 0.05)
        self.declare_parameter("maximum_range_m", 8.0)
        self.declare_parameter("quality", 255)
        self.declare_parameter("max_dt", 0.20)
        self.declare_parameter("max_speed_mps", 3.0)

        p = self.get_parameter
        self.initial_height_m = max(0.0, float(p("initial_height_m").value))
        self.minimum_range_m = max(0.01, float(p("minimum_range_m").value))
        self.maximum_range_m = max(
            self.minimum_range_m, float(p("maximum_range_m").value)
        )
        self.quality = max(0, min(255, int(p("quality").value)))
        self.max_dt = max(0.01, float(p("max_dt").value))
        self.max_speed_mps = max(0.1, float(p("max_speed_mps").value))
        publish_rate = max(1.0, float(p("publish_rate").value))

        self._flow_reference_pose = None
        self._latest_pose = None
        self._last_pose_time = 0.0
        self._last_flow_pose_stamp = None
        self.flow_pub = self.create_publisher(TwistStamped, str(p("flow_topic").value), 10)
        self.quality_pub = self.create_publisher(UInt8, str(p("quality_topic").value), 10)
        self.range_pub = self.create_publisher(Range, str(p("range_topic").value), 10)
        self.attitude_pub = self.create_publisher(
            Vector3, str(p("truth_attitude_topic").value), 10
        )
        self.create_subscription(
            PoseStamped, str(p("truth_topic").value), self._truth_cb, 20
        )
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            "Truth-derived optical-flow source enabled "
            f"({publish_rate:.0f}Hz, max_speed={self.max_speed_mps:.1f}m/s)"
        )

    @staticmethod
    def _stamp_seconds(stamp):
        # Zero is a valid simulation-clock epoch. Never replace it with a
        # monotonic timestamp: that would mix unrelated clock domains later.
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _truth_cb(self, msg: PoseStamped):
        stamp = self._stamp_seconds(msg.header.stamp)
        self._latest_pose = (stamp, msg)
        self._last_pose_time = time.monotonic()

    def _publish(self):
        now_monotonic = time.monotonic()
        pose_fresh = (
            self._latest_pose is not None
            and now_monotonic - self._last_pose_time <= self.max_dt
        )
        pose = self._latest_pose if pose_fresh else None
        pose_msg = pose[1] if pose is not None else None
        # Use the truth sample's acquisition time for the simulated sensor
        # contract.  The odometry integrator must use the same interval that
        # produced the displacement, rather than a timer callback timestamp.
        if pose is not None:
            stamp = pose_msg.header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        velocity_flu = None
        if pose is not None:
            pose_stamp = pose[0]
            if self._flow_reference_pose is None:
                self._flow_reference_pose = pose
                self._last_flow_pose_stamp = pose_stamp
            elif self._last_flow_pose_stamp != pose_stamp:
                velocity_flu = pose_displacement_to_body_velocity(
                    self._flow_reference_pose,
                    pose,
                    self.max_dt,
                    self.max_speed_mps,
                )
                # Consume the whole interval, including an invalid/outlier
                # interval, so one bad sample cannot be integrated repeatedly.
                self._flow_reference_pose = pose
                self._last_flow_pose_stamp = pose_stamp

        quality = self.quality if pose is not None and velocity_flu is not None else 0
        flow = TwistStamped()
        flow.header.stamp = stamp
        flow.header.frame_id = "base_link"
        if quality > 0:
            flow.twist.linear.x = velocity_flu[0]
            flow.twist.linear.y = velocity_flu[1]
        self.flow_pub.publish(flow)
        self.quality_pub.publish(UInt8(data=quality))

        if pose is None:
            return
        roll, pitch, yaw = quaternion_to_euler(
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        )
        self.attitude_pub.publish(Vector3(x=roll, y=pitch, z=yaw))
        height = self.initial_height_m - float(pose_msg.pose.position.z)
        distance = slant_range_to_ground(height, roll, pitch)
        if distance is None or not self.minimum_range_m <= distance <= self.maximum_range_m:
            return
        range_msg = Range()
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = "range_down"
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.10
        range_msg.min_range = self.minimum_range_m
        range_msg.max_range = self.maximum_range_m
        range_msg.range = float(distance)
        self.range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimOpticalFlowSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
