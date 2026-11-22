#!/usr/bin/env python3
"""
Publish Gazebo model truth pose as ArduPilot local NED for simulation sensors.

This node deliberately uses Gazebo truth as the source for simulated camera
and LiDAR pose. The FCU EKF output (/ap/local_pos) must not drive simulated
sensors, otherwise ExternalNav drift feeds back into the perception source.

Gazebo x500 uses FLU body/world convention for the aircraft model. ArduPilot
JSON, ExternalNav and GUIDED all expect local NED: x forward/north,
y right/east, z down, with an ArduPilot/FRD attitude convention. Gazebo's x500
pose is converted once here, so downstream nodes never mix Gazebo orientation
with NED position.
"""

import json
import math
import subprocess
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .timebase import SourceClockMapper


class GzPoseBridge(Node):
    def __init__(self):
        super().__init__("gz_pose_bridge")

        self.declare_parameter("gz_topic", "/world/competition_arena/dynamic_pose/info")
        self.declare_parameter("model_name", "drone")
        self.declare_parameter("publish_topic", "/sim/drone_pose_ned")
        self.declare_parameter("frame_id", "local_ned")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("use_initial_origin", True)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("origin_z", 0.0)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.use_initial_origin = bool(self.get_parameter("use_initial_origin").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.origin_z = float(self.get_parameter("origin_z").value)

        self.pub = self.create_publisher(PoseStamped, self.publish_topic, 10)
        self.proc = None
        self._stop = False
        self._origin_set = not self.use_initial_origin
        self._last_pub_time = 0.0
        self._last_source_pub_ns = None
        self._last_pose_time = 0.0
        self._source_clock_mapper = SourceClockMapper()

        self.thread = threading.Thread(target=self._run_gz_topic, daemon=True)
        self.thread.start()
        self.watchdog = self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f"Gazebo pose bridge: {self.gz_topic} -> {self.publish_topic}, "
            f"model={self.model_name}, rate={self.publish_rate:.0f}Hz, "
            f"initial_origin={self.use_initial_origin}"
        )

    @staticmethod
    def _gazebo_quaternion_to_ap_ned(qx, qy, qz, qw):
        """Convert Gazebo/PyBullet-style xyzw quaternion to ArduPilot NED xyzw."""
        ap_x = float(qx)
        ap_y = -float(qy)
        ap_z = -float(qz)
        ap_w = float(qw)
        norm = math.sqrt(ap_x * ap_x + ap_y * ap_y + ap_z * ap_z + ap_w * ap_w)
        if norm <= 1.0e-9:
            return 0.0, 0.0, 0.0, 1.0
        inv = 1.0 / norm
        return ap_x * inv, ap_y * inv, ap_z * inv, ap_w * inv

    def _run_gz_topic(self):
        while not self._stop and rclpy.ok():
            try:
                self.proc = subprocess.Popen(
                    ["gz", "topic", "-e", "-t", self.gz_topic, "--json-output"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                )
                assert self.proc.stdout is not None
                for line in self.proc.stdout:
                    if self._stop or not rclpy.ok():
                        break
                    self._handle_line(line)
            except Exception as exc:
                self.get_logger().warn(f"Gazebo pose bridge read failed: {exc}")
            finally:
                if self.proc is not None and self.proc.poll() is None:
                    self.proc.terminate()
                self.proc = None
            if not self._stop and rclpy.ok():
                time.sleep(1.0)

    def _handle_line(self, line: str):
        try:
            data = json.loads(line)
            pose = self._extract_model_pose(data)
            if pose is None:
                return
            source_stamp = self._extract_source_stamp(data) or self._extract_source_stamp(pose)
            now = time.monotonic()
            if source_stamp is not None:
                source_ns = source_stamp[0] * 1_000_000_000 + source_stamp[1]
                if (self._last_source_pub_ns is not None
                        and 0 <= source_ns - self._last_source_pub_ns < 1e9 / max(self.publish_rate, 1.)):
                    return
                self._last_source_pub_ns = source_ns
            elif now - self._last_pub_time < 1. / max(self.publish_rate, 1.):
                return
            self._last_pub_time = now
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            gz_x = float(position.get("x", 0.0))
            gz_y = float(position.get("y", 0.0))
            gz_z = float(position.get("z", 0.0))
        except Exception:
            return

        if not self._origin_set:
            self.origin_x = gz_x
            self.origin_y = gz_y
            self.origin_z = gz_z
            self._origin_set = True
            self.get_logger().info(
                f"Gazebo truth origin set: x={gz_x:.3f}, y={gz_y:.3f}, z={gz_z:.3f}"
            )

        msg = PoseStamped()
        if source_stamp is None:
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            ros_stamp_ns = self._source_clock_mapper.to_ros_nanoseconds(
                source_stamp, self.get_clock().now().nanoseconds
            )
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                ros_stamp_ns, 1_000_000_000
            )
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = gz_x - self.origin_x
        msg.pose.position.y = -(gz_y - self.origin_y)
        msg.pose.position.z = -(gz_z - self.origin_z)
        qx, qy, qz, qw = self._gazebo_quaternion_to_ap_ned(
            orientation.get("x", 0.0),
            orientation.get("y", 0.0),
            orientation.get("z", 0.0),
            orientation.get("w", 1.0),
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)
        self._last_pose_time = time.monotonic()

    @staticmethod
    def _extract_source_stamp(data):
        """Return a Gazebo message timestamp as ``(sec, nanosec)``.

        ``gz topic --json-output`` encodes protobuf integer fields as strings.
        Keeping the simulator timestamp is important: the transport process
        may deliver several old messages in one stdout burst, so a receive-time
        stamp would compress that burst and bias finite-difference velocities.
        """
        header = data.get("header")
        if not isinstance(header, dict):
            return None
        stamp = header.get("stamp")
        if not isinstance(stamp, dict):
            return None
        try:
            sec = int(stamp.get("sec", 0))
            nanosec = int(stamp.get("nsec", stamp.get("nanosec", 0)))
        except (TypeError, ValueError):
            return None
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return None
        return sec, nanosec

    def _extract_model_pose(self, data):
        """Return the selected model pose from Pose_V or Model JSON messages."""
        poses = data.get("pose")
        if isinstance(poses, list):
            for pose in poses:
                if pose.get("name") == self.model_name:
                    return pose
            return None

        if data.get("name") == self.model_name and isinstance(poses, dict):
            return poses

        if isinstance(poses, dict) and "position" in poses:
            return poses
        return None

    def _watchdog(self):
        if self._last_pose_time == 0.0:
            self.get_logger().warn("waiting for Gazebo truth pose...", throttle_duration_sec=5.0)
            return
        age = time.monotonic() - self._last_pose_time
        if age > 2.0:
            self.get_logger().warn(f"Gazebo truth pose stale: {age:.1f}s")

    def destroy_node(self):
        self._stop = True
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = GzPoseBridge()
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
