#!/usr/bin/env python3
"""
Feed Gazebo truth state back to ArduPilot SITL's JSON physics interface.

ArduPilot ``--model JSON`` sends servo packets to UDP port 9002 and waits for
an external simulator to return JSON state. This node provides that return path
from Gazebo so the FCU EKF and the visible Gazebo aircraft use the same state.

Input ``/sim/drone_pose_ned`` is already a complete ArduPilot local NED pose:
position is NED and orientation is an ArduPilot/FRD quaternion. This node must
not apply another Gazebo-to-NED sign flip.
"""

import json
import math
import socket
import struct
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray


GRAVITY_MSS = 9.80665


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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_quaternion_wxyz(q):
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-9:
        return 1.0, 0.0, 0.0, 0.0
    inv = 1.0 / norm
    return w * inv, x * inv, y * inv, z * inv


def quaternion_conjugate(q):
    w, x, y, z = q
    return w, -x, -y, -z


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def angular_velocity_body_from_quaternions(prev_q, curr_q, dt):
    if prev_q is None or dt <= 1.0e-6:
        return 0.0, 0.0, 0.0
    delta = normalize_quaternion_wxyz(
        quaternion_multiply(quaternion_conjugate(prev_q), curr_q)
    )
    if delta[0] < 0.0:
        delta = tuple(-v for v in delta)
    w, x, y, z = delta
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half <= 1.0e-9:
        return 2.0 * x / dt, 2.0 * y / dt, 2.0 * z / dt
    angle = 2.0 * math.atan2(sin_half, max(-1.0, min(1.0, w)))
    scale = angle / (sin_half * dt)
    return x * scale, y * scale, z * scale


def body_to_ned_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def ned_to_body(vec, roll, pitch, yaw):
    r = body_to_ned_matrix(roll, pitch, yaw)
    x, y, z = vec
    return (
        r[0][0] * x + r[1][0] * y + r[2][0] * z,
        r[0][1] * x + r[1][1] * y + r[2][1] * z,
        r[0][2] * x + r[1][2] * y + r[2][2] * z,
    )


class SITLJsonStateBridge(Node):
    def __init__(self):
        super().__init__("sitl_json_state_bridge")

        self.declare_parameter("bind_ip", "127.0.0.1")
        self.declare_parameter("bind_port", 9002)
        self.declare_parameter("pose_topic", "/sim/drone_pose_ned")
        self.declare_parameter(
            "imu_topic",
            "/world/competition_arena/model/drone/link/base_link/sensor/imu_sensor/imu",
        )
        self.declare_parameter("use_gazebo_imu", False)
        self.declare_parameter("use_gazebo_gyro", True)
        self.declare_parameter("use_gazebo_accel", False)
        self.declare_parameter("imu_stale_timeout", 0.2)
        self.declare_parameter("pose_stale_timeout", 0.5)
        self.declare_parameter("max_dt", 0.05)
        self.declare_parameter("no_time_sync", False)
        self.declare_parameter("velocity_lpf_alpha", 0.45)
        self.declare_parameter("accel_lpf_alpha", 0.35)
        self.declare_parameter("max_velocity_mps", 5.0)
        self.declare_parameter("max_accel_mps2", 18.0)
        self.declare_parameter("max_gyro_rps", 8.0)
        self.declare_parameter("send_quaternion", False)
        self.declare_parameter('synchronize_with_pose_time', False)

        self.bind_ip = str(self.get_parameter("bind_ip").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.use_gazebo_imu = bool(self.get_parameter("use_gazebo_imu").value)
        self.use_gazebo_gyro = bool(self.get_parameter("use_gazebo_gyro").value)
        self.use_gazebo_accel = bool(self.get_parameter("use_gazebo_accel").value)
        self.imu_stale_timeout = float(self.get_parameter("imu_stale_timeout").value)
        self.pose_stale_timeout = float(self.get_parameter("pose_stale_timeout").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.no_time_sync = bool(self.get_parameter("no_time_sync").value)
        self.velocity_lpf_alpha = float(self.get_parameter("velocity_lpf_alpha").value)
        self.accel_lpf_alpha = float(self.get_parameter("accel_lpf_alpha").value)
        self.max_velocity_mps = float(self.get_parameter("max_velocity_mps").value)
        self.max_accel_mps2 = float(self.get_parameter("max_accel_mps2").value)
        self.max_gyro_rps = float(self.get_parameter("max_gyro_rps").value)
        self.send_quaternion = bool(self.get_parameter("send_quaternion").value)
        self.synchronize_with_pose_time = bool(self.get_parameter('synchronize_with_pose_time').value)

        from .timebase import SensorDrivenClock
        self._sensor_clock = SensorDrivenClock()

        self._lock = threading.Lock()
        self._pose_condition = threading.Condition(self._lock)
        self._pose_source_ns = None
        self._last_pose_source_ns = None
        self._sent_pose_source_ns = None
        self._pose_time = 0.0
        self._position = (0.0, 0.0, 0.0)
        self._velocity = (0.0, 0.0, 0.0)
        self._attitude = (0.0, 0.0, 0.0)
        self._quaternion = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
        self._gyro = (0.0, 0.0, 0.0)
        self._accel_ned = (0.0, 0.0, -GRAVITY_MSS)
        self._imu_time = 0.0
        self._imu_gyro = (0.0, 0.0, 0.0)
        self._imu_accel_body = (0.0, 0.0, -GRAVITY_MSS)
        self._last_pose_position = None
        self._last_pose_quaternion = None
        self._last_pose_velocity = (0.0, 0.0, 0.0)
        self._linear_accel_ned = (0.0, 0.0, 0.0)
        self._last_pose_time = 0.0
        self._sim_time = 0.0
        self._stop = False
        self._rx_count = 0
        self._last_rx_time = 0.0
        self._last_log_time = 0.0
        self._last_debug_log_time = 0.0
        self._last_bad_packet_log_time = 0.0

        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        if self.use_gazebo_imu and self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
        self.connected_pub = self.create_publisher(Bool, "/ap/sitl_json_connected", 10)
        self.debug_pub = self.create_publisher(
            Float32MultiArray, "/ap/sitl_json_state_debug", 10
        )

        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"SITL JSON state bridge listening on {self.bind_ip}:{self.bind_port}, "
            f"pose_topic={self.pose_topic}, vel_lpf={self.velocity_lpf_alpha:.2f}, "
            f"accel_lpf={self.accel_lpf_alpha:.2f}, "
            f"use_gazebo_imu={self.use_gazebo_imu} imu_topic={self.imu_topic}, "
            f"use_gazebo_gyro={self.use_gazebo_gyro}, "
            f"use_gazebo_accel={self.use_gazebo_accel}, "
            f"send_quaternion={self.send_quaternion}"
        )

    @staticmethod
    def _limit_vector(vec, max_norm):
        if max_norm <= 0.0:
            return vec
        norm = math.sqrt(sum(v * v for v in vec))
        if norm <= max_norm or norm <= 1.0e-9:
            return vec
        scale = max_norm / norm
        return tuple(v * scale for v in vec)

    @staticmethod
    def _lpf_vector(raw, previous, alpha):
        alpha = max(0.0, min(1.0, alpha))
        return tuple(alpha * raw[i] + (1.0 - alpha) * previous[i] for i in range(3))

    @staticmethod
    def _gazebo_body_vec_to_ap(vec):
        # Gazebo x500 body is FLU; ArduPilot JSON expects FRD.
        return (float(vec[0]), -float(vec[1]), -float(vec[2]))

    def _imu_cb(self, msg: Imu):
        gyro = self._gazebo_body_vec_to_ap((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))
        accel_body = self._gazebo_body_vec_to_ap((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))
        gyro = self._limit_vector(gyro, self.max_gyro_rps)
        accel_body = self._limit_vector(accel_body, self.max_accel_mps2 + GRAVITY_MSS)

        with self._lock:
            self._imu_gyro = gyro
            self._imu_accel_body = accel_body
            self._imu_time = time.monotonic()

    def _pose_cb(self, msg):
        now = time.monotonic()
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pos = msg.pose.position
        ori = msg.pose.orientation
        position = (float(pos.x), float(pos.y), float(pos.z))
        quaternion = normalize_quaternion_wxyz(
            (float(ori.w), float(ori.x), float(ori.y), float(ori.z))
        )
        attitude = quaternion_to_euler(
            quaternion[1], quaternion[2], quaternion[3], quaternion[0]
        )

        with self._lock:
            pose_dt = now - self._last_pose_time if self._last_pose_time > 0.0 else 0.0
            if self.synchronize_with_pose_time and self._last_pose_source_ns is not None:
                if source_ns <= self._last_pose_source_ns:
                    return
                pose_dt = (source_ns - self._last_pose_source_ns) * 1e-9
            if self._last_pose_position is None or pose_dt <= 1.0e-4:
                velocity = (0.0, 0.0, 0.0)
                gyro = (0.0, 0.0, 0.0)
                accel_ned = (0.0, 0.0, -GRAVITY_MSS)
            else:
                raw_velocity = tuple(
                    (position[i] - self._last_pose_position[i]) / pose_dt
                    for i in range(3)
                )
                raw_velocity = self._limit_vector(raw_velocity, self.max_velocity_mps)
                velocity = self._lpf_vector(
                    raw_velocity, self._last_pose_velocity, self.velocity_lpf_alpha
                )
                raw_gyro = angular_velocity_body_from_quaternions(
                    self._last_pose_quaternion, quaternion, pose_dt
                )
                gyro = self._limit_vector(raw_gyro, self.max_gyro_rps)
                raw_accel = tuple(
                    (velocity[i] - self._last_pose_velocity[i]) / pose_dt
                    for i in range(3)
                )
                raw_accel = self._limit_vector(raw_accel, self.max_accel_mps2)
                linear_accel_ned = self._lpf_vector(
                    raw_accel, self._linear_accel_ned, self.accel_lpf_alpha
                )
                accel_ned = (
                    linear_accel_ned[0],
                    linear_accel_ned[1],
                    linear_accel_ned[2] - GRAVITY_MSS,
                )
                self._linear_accel_ned = linear_accel_ned

            self._position = position
            self._velocity = velocity
            self._attitude = attitude
            self._quaternion = quaternion
            self._gyro = gyro
            self._accel_ned = accel_ned
            self._pose_time = now
            self._last_pose_position = position
            self._last_pose_quaternion = quaternion
            self._last_pose_velocity = velocity
            self._last_pose_time = now
            self._pose_source_ns = source_ns
            self._last_pose_source_ns = source_ns
            self._pose_condition.notify_all()

    def _current_state(self):
        with self._lock:
            return (
                self._position,
                self._velocity,
                self._attitude,
                self._quaternion,
                self._gyro,
                self._accel_ned,
                self._pose_time,
                self._imu_gyro,
                self._imu_accel_body,
                self._imu_time,
            )

    def _publish_health(self):
        now = time.monotonic()
        connected = self._rx_count > 0 and now - self._last_rx_time < 2.0
        self.connected_pub.publish(Bool(data=connected))
        (
            position,
            velocity,
            attitude,
            _quaternion,
            gyro,
            accel_ned,
            pose_time,
            imu_gyro,
            imu_accel_body,
            imu_time,
        ) = self._current_state()
        pose_fresh = pose_time > 0.0 and now - pose_time <= self.pose_stale_timeout
        imu_fresh = (
            self.use_gazebo_imu
            and imu_time > 0.0
            and now - imu_time <= self.imu_stale_timeout
        )
        accel_body = (
            imu_accel_body
            if imu_fresh and self.use_gazebo_accel
            else ned_to_body(accel_ned, *attitude)
        )
        use_gyro = imu_gyro if imu_fresh and self.use_gazebo_gyro else gyro

        debug = Float32MultiArray()
        debug.data = [
            float(position[0]), float(position[1]), float(position[2]),
            float(velocity[0]), float(velocity[1]), float(velocity[2]),
            float(attitude[0]), float(attitude[1]), float(attitude[2]),
            float(use_gyro[0]), float(use_gyro[1]), float(use_gyro[2]),
            float(accel_body[0]), float(accel_body[1]), float(accel_body[2]),
            1.0 if pose_fresh else 0.0,
            1.0 if imu_fresh else 0.0,
            1.0 if connected else 0.0,
        ]
        self.debug_pub.publish(debug)

        if now - self._last_debug_log_time >= 5.0:
            self._last_debug_log_time = now
            self.get_logger().info(
                "JSON状态 "
                f"connected={connected} pose_fresh={pose_fresh} imu_fresh={imu_fresh} "
                f"pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) "
                f"vel=({velocity[0]:.2f},{velocity[1]:.2f},{velocity[2]:.2f}) "
                f"rpy=({math.degrees(attitude[0]):.1f},{math.degrees(attitude[1]):.1f},"
                f"{math.degrees(attitude[2]):.1f})deg "
                f"gyro=({use_gyro[0]:.2f},{use_gyro[1]:.2f},{use_gyro[2]:.2f}) "
                f"accel_body=({accel_body[0]:.2f},{accel_body[1]:.2f},{accel_body[2]:.2f})"
            )

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.bind_port))
        sock.settimeout(0.1)

        while not self._stop and rclpy.ok():
            try:
                data, address = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            frame_rate = self._parse_frame_rate(data)
            if frame_rate is None:
                now = time.monotonic()
                if now - self._last_bad_packet_log_time > 5.0:
                    self._last_bad_packet_log_time = now
                    self.get_logger().warn(
                        f"ignored non-ArduPilot JSON servo packet from {address} "
                        f"len={len(data)}"
                    )
                continue

            dt = 1.0 / frame_rate if frame_rate > 0 else 0.0025
            if self.max_dt > 0.0:
                dt = min(dt, self.max_dt)
            if self.synchronize_with_pose_time:
                with self._pose_condition:
                    while not self._stop and (
                        self._pose_source_ns is None or self._pose_source_ns == self._sent_pose_source_ns
                    ):
                        self._pose_condition.wait(timeout=.1)
                    if self._stop:
                        break
                    source_ns = self._pose_source_ns
                    self._sent_pose_source_ns = source_ns
                measured_time = self._sensor_clock.advance(source_ns)
                if measured_time is None:
                    continue
                dt = measured_time - self._sim_time
                self._sim_time = measured_time
            else:
                self._sim_time += dt

            payload = self._build_state_json(dt)
            try:
                sock.sendto(payload, address)
            except OSError as exc:
                self.get_logger().warn(f"SITL JSON send failed: {exc}", throttle_duration_sec=2.0)
                continue

            self._rx_count += 1
            self._last_rx_time = time.monotonic()
            if self._last_rx_time - self._last_log_time > 5.0:
                self._last_log_time = self._last_rx_time
                self.get_logger().info(
                    f"SITL JSON connected: frames={self._rx_count}, "
                    f"rate={frame_rate or 0}Hz, sim_time={self._sim_time:.1f}s"
                )

        sock.close()

    def _parse_frame_rate(self, data):
        if len(data) < 8:
            return None
        try:
            magic, frame_rate = struct.unpack_from("<HH", data, 0)
        except struct.error:
            return None
        if magic == 18458 and len(data) == 40:
            return int(frame_rate)
        if magic == 29569 and len(data) == 72:
            return int(frame_rate)
        return None

    def _build_state_json(self, dt):
        (
            position,
            velocity,
            attitude,
            quaternion,
            gyro,
            accel_ned,
            pose_time,
            imu_gyro,
            imu_accel_body,
            imu_time,
        ) = self._current_state()
        pose_fresh = pose_time > 0.0 and time.monotonic() - pose_time <= self.pose_stale_timeout
        imu_fresh = (
            self.use_gazebo_imu
            and imu_time > 0.0
            and time.monotonic() - imu_time <= self.imu_stale_timeout
        )
        if not pose_fresh:
            velocity = (0.0, 0.0, 0.0)
            gyro = (0.0, 0.0, 0.0)
            accel_ned = (0.0, 0.0, -GRAVITY_MSS)

        if imu_fresh and self.use_gazebo_gyro:
            gyro = imu_gyro
        accel_body = (
            imu_accel_body
            if imu_fresh and self.use_gazebo_accel
            else ned_to_body(accel_ned, *attitude)
        )

        state = {
            "timestamp": self._sim_time,
            "imu": {
                "gyro": [gyro[0], gyro[1], gyro[2]],
                "accel_body": [accel_body[0], accel_body[1], accel_body[2]],
            },
            "position": [position[0], position[1], position[2]],
            "attitude": [attitude[0], attitude[1], attitude[2]],
            "velocity": [velocity[0], velocity[1], velocity[2]],
            "no_time_sync": 1 if self.no_time_sync else 0,
        }
        if self.send_quaternion:
            state["quaternion"] = [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        return (json.dumps(state, separators=(",", ":")) + "\n").encode("ascii")

    def destroy_node(self):
        self._stop = True
        with self._pose_condition:
            self._pose_condition.notify_all()
        super().destroy_node()


def main():
    rclpy.init()
    node = SITLJsonStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # rclpy may invalidate a subscription while launch is shutting the
        # shared context down. Suppress only that shutdown race; real runtime
        # failures must still surface.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
