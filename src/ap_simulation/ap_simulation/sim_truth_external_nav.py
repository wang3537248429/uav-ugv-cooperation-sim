#!/usr/bin/env python3
"""
Publish Gazebo truth pose as /ap/external_nav/vision_pose for closed-loop simulation.

This is a simulation-only ExternalNav source. It lets the Gazebo + SITL full
mission validate controller and state-machine behavior without a mapping or
physical positioning sensor.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


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


class SimTruthExternalNav(Node):
    def __init__(self):
        super().__init__("sim_truth_external_nav")

        self.declare_parameter("truth_topic", "/sim/drone_pose_ned")
        self.declare_parameter("input_mode", "truth")
        self.declare_parameter("odometry_topic", "/odometry/optical_flow")
        self.declare_parameter("vision_pose_topic", "/ap/external_nav/vision_pose")
        self.declare_parameter("ready_topic", "/ap/external_nav/ready")
        self.declare_parameter("covariance_topic", "/ap/external_nav/covariance")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("truth_stale_timeout", 0.5)
        self.declare_parameter("cov_pos_xy", 0.02)
        self.declare_parameter("cov_pos_z", 0.04)
        self.declare_parameter("cov_ang_yaw", 0.03)
        self.declare_parameter("cov_ang_rp", 100.0)
        self.declare_parameter("roll_pitch_source", "flat")  # flat|truth|fcu
        self.declare_parameter("yaw_source", "truth")  # truth|fcu|zero

        self.truth_topic = str(self.get_parameter("truth_topic").value)
        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        if self.input_mode not in ("truth", "odometry"):
            self.get_logger().warn(f"input_mode={self.input_mode!r}无效, 回退truth")
            self.input_mode = "truth"
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.vision_pose_topic = str(self.get_parameter("vision_pose_topic").value)
        self.ready_topic = str(self.get_parameter("ready_topic").value)
        self.covariance_topic = str(self.get_parameter("covariance_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.truth_stale_timeout = float(self.get_parameter("truth_stale_timeout").value)
        self.cov_xy = float(self.get_parameter("cov_pos_xy").value)
        self.cov_z = float(self.get_parameter("cov_pos_z").value)
        self.cov_yaw = float(self.get_parameter("cov_ang_yaw").value)
        self.cov_rp = float(self.get_parameter("cov_ang_rp").value)
        self.roll_pitch_source = str(self.get_parameter("roll_pitch_source").value).lower()
        self.yaw_source = str(self.get_parameter("yaw_source").value).lower()
        if self.roll_pitch_source not in ("flat", "truth", "fcu"):
            self.get_logger().warn(
                f"roll_pitch_source={self.roll_pitch_source}无效, 回退flat"
            )
            self.roll_pitch_source = "flat"
        self._last_truth = None
        self._last_truth_time = 0.0
        self._last_attitude_time = 0.0
        self._roll_rad = 0.0
        self._pitch_rad = 0.0
        self._yaw_rad = 0.0
        self._odometry_covariance = None

        self.create_subscription(PoseStamped, self.truth_topic, self._truth_cb, 10)
        if self.input_mode == "odometry":
            self.create_subscription(Odometry, self.odometry_topic, self._odometry_cb, 10)
        self.create_subscription(Vector3, "/ap/roll_pitch_yaw", self._attitude_cb, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.vision_pose_topic, 10)
        self.ready_pub = self.create_publisher(Bool, self.ready_topic, 10)
        self.cov_pub = self.create_publisher(Float32, self.covariance_topic, 10)
        self.create_timer(1.0 / max(publish_rate, 1.0), self._publish)

        self.get_logger().info(
            f"Simulation truth ExternalNav: {self.truth_topic} -> {self.vision_pose_topic}, "
            f"input={self.input_mode}, rate={publish_rate:.0f}Hz, yaw_source={self.yaw_source}, "
            f"roll_pitch_source={self.roll_pitch_source}"
        )

    def _truth_cb(self, msg: PoseStamped):
        if self.input_mode != "truth":
            return
        self._last_truth = msg
        self._last_truth_time = time.monotonic()

    def _odometry_cb(self, msg: Odometry):
        if self.input_mode != "odometry":
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._last_truth = pose
        self._odometry_covariance = list(msg.pose.covariance)
        self._last_truth_time = time.monotonic()

    def _attitude_cb(self, msg: Vector3):
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            return
        self._roll_rad = float(msg.x)
        self._pitch_rad = float(msg.y)
        self._yaw_rad = float(msg.z)
        self._last_attitude_time = time.monotonic()

    def _publish(self):
        now = time.monotonic()
        if self._last_truth is None or now - self._last_truth_time > self.truth_stale_timeout:
            self.ready_pub.publish(Bool(data=False))
            return

        src = self._last_truth
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "local_ned"
        pose.pose.pose.position.x = float(src.pose.position.x)
        pose.pose.pose.position.y = float(src.pose.position.y)
        pose.pose.pose.position.z = float(src.pose.position.z)

        src_q = src.pose.orientation
        truth_roll, truth_pitch, truth_yaw = quaternion_to_euler(
            src_q.x, src_q.y, src_q.z, src_q.w
        )
        if self.roll_pitch_source == "truth":
            roll = truth_roll
            pitch = truth_pitch
        elif self.roll_pitch_source == "fcu" and now - self._last_attitude_time <= 1.0:
            roll = self._roll_rad
            pitch = self._pitch_rad
        else:
            roll = 0.0
            pitch = 0.0

        if self.yaw_source == "fcu" and now - self._last_attitude_time <= 1.0:
            yaw = self._yaw_rad
        elif self.yaw_source == "zero":
            yaw = 0.0
        else:
            yaw = truth_yaw

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        if self.input_mode == "odometry" and self._odometry_covariance is not None:
            cov[0] = self._odometry_covariance[0]
            cov[7] = self._odometry_covariance[7]
            cov[14] = self._odometry_covariance[14]
            cov[21] = self.cov_rp
            cov[28] = self.cov_rp
            cov[35] = self.cov_yaw
        else:
            cov[0] = self.cov_xy
            cov[7] = self.cov_xy
            cov[14] = self.cov_z
            cov[21] = self.cov_rp
            cov[28] = self.cov_rp
            cov[35] = self.cov_yaw
        pose.pose.covariance = cov

        self.pose_pub.publish(pose)
        self.ready_pub.publish(Bool(data=True))
        self.cov_pub.publish(Float32(data=cov[0]))


def main():
    rclpy.init()
    node = SimTruthExternalNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
