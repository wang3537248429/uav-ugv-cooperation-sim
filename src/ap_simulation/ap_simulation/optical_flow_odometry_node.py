#!/usr/bin/env python3
"""ROS 2 adapter for a real optical-flow camera and downward range sensor.

Inputs use the common ROS contracts:
  ``geometry_msgs/TwistStamped``: body-frame velocity in m/s;
  ``std_msgs/UInt8``: camera quality [0, 255];
  ``sensor_msgs/Range``: downward range in metres;
  ``geometry_msgs/Vector3``: roll, pitch, yaw in radians.

Output is ``nav_msgs/Odometry`` plus ``nav_msgs/Path`` in ``local_ned``.  The
node is usable on the real aircraft and can be enabled independently of the
competition state machine.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt8

from .optical_flow_odometry import (
    FlowTimestampMapper,
    OpticalFlowIntegrator,
    OpticalFlowSample,
    attitude_rpy_to_ned_flu_quaternion,
    ned_to_body_flu,
)


class OpticalFlowOdometryNode(Node):
    def __init__(self):
        super().__init__("optical_flow_odometry")
        self.declare_parameter("flow_topic", "/optical_flow/velocity")
        self.declare_parameter("quality_topic", "/optical_flow/quality")
        self.declare_parameter("range_topic", "/range/down")
        self.declare_parameter("attitude_topic", "/ap/roll_pitch_yaw")
        self.declare_parameter("odom_topic", "/odometry/optical_flow")
        self.declare_parameter("path_topic", "/odometry/optical_flow_path")
        self.declare_parameter("frame_id", "local_ned")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("min_quality", 80)
        self.declare_parameter("max_dt", 0.20)
        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("velocity_alpha", 0.35)
        self.declare_parameter("path_length", 500)
        self.declare_parameter("sensor_stale_timeout", 0.50)
        self.declare_parameter("attitude_stale_timeout", 0.50)

        p = self.get_parameter
        self.frame_id = str(p("frame_id").value)
        self.child_frame_id = str(p("child_frame_id").value)
        self.path_length = max(10, int(p("path_length").value))
        self.sensor_stale_timeout = max(0.05, float(p("sensor_stale_timeout").value))
        self.attitude_stale_timeout = max(
            0.05, float(p("attitude_stale_timeout").value)
        )
        self.integrator = OpticalFlowIntegrator(
            min_quality=int(p("min_quality").value), max_dt=float(p("max_dt").value),
            max_speed=float(p("max_speed").value), velocity_alpha=float(p("velocity_alpha").value))
        self._quality = 0
        self._last_quality_time = 0.0
        self._range_m = 0.0
        self._last_range_time = 0.0
        self._attitude_rpy = (0.0, 0.0, 0.0)
        self._last_attitude_time = 0.0
        self._last_flow_stamp = 0.0
        self._flow_timestamp_mapper = FlowTimestampMapper()
        self.odom_pub = self.create_publisher(Odometry, str(p("odom_topic").value), 10)
        self.path_pub = self.create_publisher(Path, str(p("path_topic").value), 2)
        self._path = Path()
        self._path.header.frame_id = self.frame_id
        self.create_subscription(TwistStamped, str(p("flow_topic").value), self._flow_cb, 20)
        self.create_subscription(UInt8, str(p("quality_topic").value), self._quality_cb, 20)
        self.create_subscription(Range, str(p("range_topic").value), self._range_cb, 20)
        self.create_subscription(Vector3, str(p("attitude_topic").value), self._attitude_cb, 20)
        self.create_timer(1.0, self._watchdog)

    def _quality_cb(self, msg: UInt8):
        self._quality = int(msg.data)
        self._last_quality_time = time.monotonic()

    def _range_cb(self, msg: Range):
        if math.isfinite(msg.range):
            self._range_m = float(msg.range)
            self._last_range_time = time.monotonic()

    def _attitude_cb(self, msg: Vector3):
        values = (float(msg.x), float(msg.y), float(msg.z))
        if all(math.isfinite(value) for value in values):
            self._attitude_rpy = values
            self._last_attitude_time = time.monotonic()

    def _flow_cb(self, msg: TwistStamped):
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_quality_time > self.sensor_stale_timeout:
            return
        if now_monotonic - self._last_range_time > self.sensor_stale_timeout:
            return
        if now_monotonic - self._last_attitude_time > self.attitude_stale_timeout:
            return
        source_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        stamp = self._flow_timestamp_mapper.to_monotonic_seconds(
            source_stamp, now_monotonic
        )
        ros_now = self.get_clock().now()
        output_stamp = ros_now.to_msg()
        if math.isfinite(source_stamp) and source_stamp > 0.0:
            # Convert sensor age back into the ROS clock domain instead of
            # copying a source stamp that may use another epoch or have reset.
            age_ns = int(max(0.0, now_monotonic - stamp) * 1.0e9)
            output_ns = max(0, ros_now.nanoseconds - age_ns)
            output_stamp.sec, output_stamp.nanosec = divmod(
                output_ns, 1_000_000_000
            )
        result = self.integrator.update(
            OpticalFlowSample(stamp, (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
                              self._quality, self._range_m), self._attitude_rpy)
        self._last_flow_stamp = now_monotonic
        if result is None:
            return
        odom = Odometry()
        # Preserve the sensor acquisition time. Using the callback time adds
        # executor scheduling latency and makes this estimate difficult to
        # fuse with the truth/attitude streams that use the flow timestamp.
        odom.header.stamp = output_stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = result.position_ned
        orientation = attitude_rpy_to_ned_flu_quaternion(*self._attitude_rpy)
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = orientation
        velocity_body = ned_to_body_flu(result.velocity_ned, *self._attitude_rpy)
        (
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ) = velocity_body
        for i in (0, 7, 14):
            odom.pose.covariance[i] = result.covariance
        for i in (21, 28, 35):
            odom.pose.covariance[i] = 1.0e6
        for i in (0, 7, 14):
            odom.twist.covariance[i] = result.covariance
        self.odom_pub.publish(odom)
        self._path.header.stamp = output_stamp
        self._path.poses.append(odom_to_pose(odom))
        if len(self._path.poses) > self.path_length:
            self._path.poses = self._path.poses[-self.path_length:]
        self.path_pub.publish(self._path)

    def _watchdog(self):
        if self._last_flow_stamp and time.monotonic() - self._last_flow_stamp > 1.0:
            self.get_logger().warn("optical-flow stream is stale", throttle_duration_sec=5.0)


def odom_to_pose(odom: Odometry):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header = odom.header
    pose.pose = odom.pose.pose
    return pose


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowOdometryNode()
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
