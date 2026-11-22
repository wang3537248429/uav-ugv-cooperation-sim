#!/usr/bin/env python3
"""ArduPilot SITL to ROS 2 MAVLink bridge.

Connection: MAVLink 2 over SITL TCP (default ``tcp:127.0.0.1:5760``).
职责:
  1. 连接SITL, 等待HEARTBEAT
  2. 读取SITL状态 (模式/解锁/高度/电压/姿态)
  3. 发布 ROS 2 状态话题
  4. 订阅 ROS 2 控制话题
  5. 发送 MAVLink 指令 (ARM/DISARM/模式切换/GUIDED起飞/RC override/降落)
"""

import math
import time
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from .bridge_config import (
    BRIDGE_PARAMETER_DEFAULTS,
    DEFAULT_ORIGIN_ALT_MSL,
    ORIGIN_LAT,
    ORIGIN_LON,
)
from .bridge_config import DEFAULT_SITL_URL  # noqa: F401 (compatibility export)
from .optical_flow import velocity_to_flow_rates

from std_msgs.msg import Bool, String, Float32, Float32MultiArray, UInt8
from geometry_msgs.msg import Vector3, Twist, PoseWithCovarianceStamped, Point, TwistStamped
from sensor_msgs.msg import Joy, Range

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402


def quaternion_to_euler(qx, qy, qz, qw):
    """标准四元数→欧拉角 (ROS ENU: roll/pitch/yaw in rad)"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Euler angles to quaternion in MAVLink w,x,y,z order."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def wrap_angle_rad(angle):
    """Normalize angle to [-pi, pi] in radians."""
    return math.atan2(math.sin(angle), math.cos(angle))


class APMavlinkBridge(Node):
    """AP飞控 MAVLink 桥节点"""

    def __init__(self):
        super().__init__("ap_mavlink_bridge")

        # 参数声明
        for name, default in BRIDGE_PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

        self.connection_url = self.get_parameter("connection_url").value
        self.gcs_output_url = str(self.get_parameter("gcs_output_url").value).strip()
        self.gcs_forward_commands = bool(
            self.get_parameter("gcs_forward_commands").value
        )
        self.simulation_connection = str(self.connection_url).startswith("tcp:")
        self.allow_hardware_connection = bool(
            self.get_parameter("allow_hardware_connection").value
        )
        if not self.simulation_connection and not self.allow_hardware_connection:
            raise ValueError(
                "非TCP飞控连接需要显式设置 allow_hardware_connection=true"
            )
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)
        self.enable_external_nav = self.get_parameter(
            "enable_external_nav"
        ).value
        self.localization_backend = str(
            self.get_parameter("localization_backend").value
        ).strip().lower()
        if self.localization_backend not in (
            "external_nav", "fcu_local", "optical_flow_ekf"
        ):
            self.get_logger().warn(
                f"localization_backend={self.localization_backend!r}无效, "
                "回退external_nav"
            )
            self.localization_backend = "external_nav"
        if self.localization_backend == "optical_flow_ekf" and self.enable_external_nav:
            self.get_logger().info(
                "optical_flow_ekf已选择，停用VISION_POSITION_ESTIMATE ExternalNav输入"
            )
            self.enable_external_nav = False
        self.cov_xy = float(self.get_parameter("vision_pose_cov_pos_xy").value)
        self.cov_z = float(self.get_parameter("vision_pose_cov_pos_z").value)
        self.cov_vel = float(self.get_parameter("vision_pose_cov_vel").value)
        self.cov_yaw = float(self.get_parameter("vision_pose_cov_ang_yaw").value)
        self.cov_rp = float(self.get_parameter("vision_pose_cov_ang_rp").value)
        self.set_home_on_start = bool(self.get_parameter('set_home_on_start').value)
        self.ap_viso_yaw_noise_rad = max(0.05, float(self.get_parameter("ap_viso_yaw_noise_rad").value))
        self.set_ekf_origin_on_start = bool(self.get_parameter("set_ekf_origin_on_start").value)
        self.ekf_origin_retry_period = float(self.get_parameter("ekf_origin_retry_period").value)
        self.force_vision_flat_attitude = bool(self.get_parameter("force_vision_flat_attitude").value)
        self.vision_yaw_source = str(self.get_parameter("vision_yaw_source").value).strip().lower()
        if self.vision_yaw_source not in ("external_nav", "fcu_attitude"):
            self.get_logger().warn(
                f"未知 vision_yaw_source={self.vision_yaw_source!r}, "
                "回退为 external_nav"
            )
            self.vision_yaw_source = "external_nav"
        self.vision_yaw_attitude_max_age = max(
            0.05,
            float(self.get_parameter("vision_yaw_attitude_max_age").value),
        )
        self.vision_pose_jump_guard_enabled = bool(
            self.get_parameter("vision_pose_jump_guard_enabled").value
        )
        self.vision_pose_jump_max_m = max(
            0.01,
            float(self.get_parameter("vision_pose_jump_max_m").value),
        )
        self.vision_pose_jump_max_mps = max(
            0.01,
            float(self.get_parameter("vision_pose_jump_max_mps").value),
        )
        self.vision_pose_jump_hold_timeout = max(
            0.1,
            float(self.get_parameter("vision_pose_jump_hold_timeout").value),
        )
        self.send_vision_speed_estimate = bool(self.get_parameter("send_vision_speed_estimate").value)
        self.vision_speed_max_mps = float(self.get_parameter("vision_speed_max_mps").value)
        self.vision_speed_jump_mps = float(self.get_parameter("vision_speed_jump_mps").value)
        self.vision_speed_min_dt = float(self.get_parameter("vision_speed_min_dt").value)
        self.vision_speed_max_dt = float(self.get_parameter("vision_speed_max_dt").value)
        self.vision_speed_lpf_alpha = float(self.get_parameter("vision_speed_lpf_alpha").value)
        self.enable_native_optical_flow = bool(
            self.get_parameter("enable_native_optical_flow").value
        )
        self.optical_flow_input_topic = str(
            self.get_parameter("optical_flow_input_topic").value
        )
        self.optical_flow_quality_topic = str(
            self.get_parameter("optical_flow_quality_topic").value
        )
        self.distance_sensor_topic = str(
            self.get_parameter("distance_sensor_topic").value
        )
        self.optical_flow_min_quality = max(
            0, min(255, int(self.get_parameter("optical_flow_min_quality").value))
        )
        self.optical_flow_min_height = max(
            0.01, float(self.get_parameter("optical_flow_min_height").value)
        )
        self.optical_flow_max_height = max(
            self.optical_flow_min_height,
            float(self.get_parameter("optical_flow_max_height").value),
        )
        self.optical_flow_min_dt = max(
            0.001, float(self.get_parameter("optical_flow_min_dt").value)
        )
        self.optical_flow_max_dt = max(
            self.optical_flow_min_dt,
            float(self.get_parameter("optical_flow_max_dt").value),
        )
        self.optical_flow_max_rate_rad_s = max(
            0.1, float(self.get_parameter("optical_flow_max_rate_rad_s").value)
        )
        self.optical_flow_sensor_id = max(
            0, min(255, int(self.get_parameter("optical_flow_sensor_id").value))
        )
        self.distance_sensor_id = max(
            0, min(255, int(self.get_parameter("distance_sensor_id").value))
        )
        self.distance_sensor_min_m = max(
            0.01, float(self.get_parameter("distance_sensor_min_m").value)
        )
        self.distance_sensor_max_m = max(
            self.distance_sensor_min_m,
            float(self.get_parameter("distance_sensor_max_m").value),
        )
        self.distance_sensor_orientation = int(
            self.get_parameter("distance_sensor_orientation").value
        )
        self.optical_flow_x_sign = math.copysign(
            1.0, float(self.get_parameter("optical_flow_x_sign").value) or 1.0
        )
        self.optical_flow_y_sign = math.copysign(
            1.0, float(self.get_parameter("optical_flow_y_sign").value) or 1.0
        )
        self.optical_flow_sensor_yaw_rad = float(
            self.get_parameter("optical_flow_sensor_yaw_rad").value
        )
        self.external_nav_diagnostics = bool(self.get_parameter("external_nav_diagnostics").value)
        self.external_nav_diagnostics_period = float(
            self.get_parameter("external_nav_diagnostics_period").value
        )
        self.external_nav_baro_z_fusion = bool(
            self.get_parameter("external_nav_baro_z_fusion").value
        )
        self.external_nav_baro_z_max_age = max(
            0.1, float(self.get_parameter("external_nav_baro_z_max_age").value)
        )
        self.guided_attitude_climb_full_scale = float(
            self.get_parameter("guided_attitude_climb_full_scale").value
        )
        self.guided_attitude_hover_thrust = max(
            0.0, min(1.0, float(self.get_parameter("guided_attitude_hover_thrust").value))
        )
        self.guided_attitude_thrust_min = max(
            0.0, min(1.0, float(self.get_parameter("guided_attitude_thrust_min").value))
        )
        self.guided_attitude_thrust_max = max(
            0.0, min(1.0, float(self.get_parameter("guided_attitude_thrust_max").value))
        )
        if self.guided_attitude_thrust_min > self.guided_attitude_thrust_max:
            self.get_logger().warn(
                "guided_attitude_thrust_min 大于 max, 已交换两个限幅值"
            )
            self.guided_attitude_thrust_min, self.guided_attitude_thrust_max = (
                self.guided_attitude_thrust_max,
                self.guided_attitude_thrust_min,
            )
        self.guided_yaw_lock_enabled = bool(
            self.get_parameter("guided_yaw_lock_enabled").value
        )
        self.guided_yaw_lock_rad = wrap_angle_rad(
            float(self.get_parameter("guided_yaw_lock_rad").value)
        )
        self.invert_guided_position_y = bool(
            self.get_parameter("invert_guided_position_y").value
        )
        self.invert_guided_velocity_y = bool(
            self.get_parameter("invert_guided_velocity_y").value
        )
        self.ap_angle_max_cd = float(self.get_parameter("ap_angle_max_cd").value)
        self.ap_psc_angle_max_deg = float(
            self.get_parameter("ap_psc_angle_max_deg").value
        )
        self.ap_guid_options = int(self.get_parameter("ap_guid_options").value)
        self.ap_loit_speed_cms = float(self.get_parameter("ap_loit_speed_cms").value)
        self.ap_loit_acc_max_cmss = float(self.get_parameter("ap_loit_acc_max_cmss").value)
        self.ap_wpnav_speed_cms = float(self.get_parameter("ap_wpnav_speed_cms").value)
        self.ap_wpnav_speed_up_cms = float(self.get_parameter("ap_wpnav_speed_up_cms").value)
        self.ap_wpnav_speed_dn_cms = float(self.get_parameter("ap_wpnav_speed_dn_cms").value)
        self.ap_wpnav_accel_cmss = float(self.get_parameter("ap_wpnav_accel_cmss").value)
        self.ap_wpnav_accel_z_cmss = float(self.get_parameter("ap_wpnav_accel_z_cmss").value)
        self.ap_psc_posxy_p = float(self.get_parameter("ap_psc_posxy_p").value)
        self.ap_psc_velxy_p = float(self.get_parameter("ap_psc_velxy_p").value)
        self.ap_psc_velxy_i = float(self.get_parameter("ap_psc_velxy_i").value)
        self.ap_psc_velxy_d = float(self.get_parameter("ap_psc_velxy_d").value)
        self.ap_mot_thst_hover = max(
            0.20, min(0.80, float(self.get_parameter("ap_mot_thst_hover").value))
        )
        self.nav_takeoff_min_altitude = max(
            0.1, float(self.get_parameter("nav_takeoff_min_altitude").value)
        )
        self.allow_low_alt_nav_takeoff = bool(
            self.get_parameter("allow_low_alt_nav_takeoff").value
        )
        self.mavlink_read_period = float(self.get_parameter("mavlink_read_period").value)
        self.raw_controller_hz = int(self.get_parameter("raw_controller_hz").value)
        self.force_disarm = bool(self.get_parameter("force_disarm").value)

        self.master = None
        self.gcs_link = None
        self._last_msg_time = time.monotonic()
        self._reconnecting = False

        # ========== 连接飞控 (带重试, TCP连接容忍SITL初始化延迟) ==========
        self.get_logger().info(f"正在连接 ArduPilot SITL: {self.connection_url}")

        max_retries = 15
        retry_delay = 5.0

        for attempt in range(max_retries):
            if not rclpy.ok():
                raise ExternalShutdownException()
            try:
                self.master = self._open_mavlink_connection()

                self.get_logger().info("等待飞控 HEARTBEAT...")
                hb = self._wait_heartbeat(timeout=30)
                if hb is None:
                    raise TimeoutError("HEARTBEAT超时 (30s)")

                self._handle_heartbeat(hb)

                self.get_logger().info(
                    f"飞控已连接! target_system={self.master.target_system}, "
                    f"target_component={self.master.target_component}"
                )

                # 设置机架参数 (可能需在连接后设置, 避免--defaults触发重启)
                if self.simulation_connection:
                    self._setup_critical_params()
                else:
                    self.get_logger().info(
                        "硬件MAVLink连接已启用，跳过SITL专用参数写入"
                    )

                break
            except Exception as e:
                self.get_logger().warn(f"飞控连接失败 (尝试 {attempt+1}/{max_retries}): {e}")
                # 确保关闭已有连接, 否则SITL单连接模式会拒绝重试
                self._close_master()
                if attempt < max_retries - 1:
                    retry_deadline = time.monotonic() + retry_delay
                    while time.monotonic() < retry_deadline:
                        if not rclpy.ok():
                            raise ExternalShutdownException()
                        time.sleep(0.1)
                else:
                    self.get_logger().error(f"飞控连接最终失败: {e}")
                    raise

        if self.gcs_output_url:
            try:
                self.gcs_link = mavutil.mavlink_connection(
                    self.gcs_output_url,
                    source_system=self.source_system,
                    source_component=self.source_component,
                )
                self.get_logger().info(
                    f"MAVLink 地面站转发已启用: {self.gcs_output_url}"
                )
            except Exception as error:
                self.gcs_link = None
                self.get_logger().error(
                    f"无法打开 MAVLink 地面站转发 {self.gcs_output_url}: {error}"
                )

        # 请求飞控数据流 (无--defaults时不触发参数重载)
        self.request_telemetry()

        # ========== 状态发布者 ==========
        self.connected_pub = self.create_publisher(Bool, "/ap/connected", 10)
        self.home_ready_pub = self.create_publisher(Bool, '/ap/home_ready', 10)
        self.landed_pub = self.create_publisher(Bool, '/ap/landed', 10)
        self.mode_pub = self.create_publisher(String, "/ap/mode", 10)
        self.armed_pub = self.create_publisher(Bool, "/ap/armed", 10)
        self.alt_pub = self.create_publisher(Float32, "/ap/relative_alt", 10)
        self.batt_pub = self.create_publisher(Float32, "/ap/battery_voltage", 10)
        self.rpy_pub = self.create_publisher(Vector3, "/ap/roll_pitch_yaw", 10)
        self.ekf_ok_pub = self.create_publisher(Bool, "/ap/ekf_ok", 10)
        self.prearm_ok_pub = self.create_publisher(Bool, "/ap/prearm_ok", 10)
        self.system_status_pub = self.create_publisher(String, "/ap/system_status", 10)
        self.local_pos_pub = self.create_publisher(Point, "/ap/local_pos", 10)  # NED位置
        self.local_velocity_pub = self.create_publisher(Twist, "/ap/local_velocity", 10)
        self.motor_pwm_pub = self.create_publisher(Float32MultiArray, "/ap/motor_pwm", 10)

        # ========== 控制订阅者 ==========
        self.create_subscription(String, "/ap/cmd/set_mode", self.set_mode_cb, 10)
        self.create_subscription(Bool, "/ap/cmd/arm", self.arm_cb, 10)
        self.create_subscription(Bool, "/ap/cmd/land", self.land_cb, 10)
        self.create_subscription(Float32, "/ap/cmd/takeoff_alt", self.takeoff_cb, 10)
        self.create_subscription(Joy, "/ap/cmd/rc_override", self.rc_override_cb, 10)
        self.create_subscription(Bool, "/ap/cmd/release_rc_override", self.release_rc_override_cb, 10)
        self.create_subscription(Twist, "/ap/cmd/body_velocity", self.body_velocity_cb, 10)
        self.create_subscription(Point, "/ap/cmd/guided_position", self.guided_position_cb, 10)
        self.create_subscription(Twist, "/ap/cmd/guided_velocity", self.guided_velocity_cb, 10)
        self.create_subscription(
            Float32, "/ap/cmd/guided_level_climb_rate", self.guided_level_climb_cb, 10
        )

        # 仿真 ExternalNav 位姿订阅 (VISION_POSITION_ESTIMATE 桥接)
        if self.enable_external_nav:
            self.create_subscription(
                PoseWithCovarianceStamped, "/ap/external_nav/vision_pose", self.vision_pose_cb, 10
            )
            self.get_logger().info(
                "ExternalNav bridge enabled: /ap/external_nav/vision_pose"
            )

        if self.enable_native_optical_flow:
            self.create_subscription(
                TwistStamped,
                self.optical_flow_input_topic,
                self.native_optical_flow_cb,
                20,
            )
            self.create_subscription(
                UInt8,
                self.optical_flow_quality_topic,
                self.native_optical_flow_quality_cb,
                20,
            )
            self.create_subscription(
                Range,
                self.distance_sensor_topic,
                self.native_distance_sensor_cb,
                20,
            )
            self.get_logger().info(
                "Native optical flow bridge enabled: "
                f"{self.optical_flow_input_topic} + {self.distance_sensor_topic}"
            )

        # 所有MAVLink操作均在主线程定时器中执行, 无需锁

        # ========== RC摇杆解锁状态机 (运行在主线程定时器中, 避免线程竞争) ==========
        # 状态: idle → stage1(油门低+方向右, 1.5s) → stage2(油门低+方向中, 1.0s) → done
        self._arm_sm = "idle"           # idle | stage1 | stage2 | done
        self._arm_sm_deadline = 0.0     # 当前阶段截止时间 (monotonic)
        self._arm_was_reading = True

        # ========== 状态变量 ==========
        self.current_mode = "UNKNOWN"
        self.is_armed = False
        self.relative_alt = 0.0
        self._relative_alt_time = 0.0
        self.absolute_alt_msl = None
        self.battery_voltage = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ekf_ok = False
        self.last_vision_pose = None  # (x,y,z,roll,pitch,yaw) local NED
        self._last_vision_pose_sent = None  # (monotonic_time,x,y,z)
        self._vision_pose_jump_hold_until = 0.0
        self._last_vision_speed_sample = None  # (monotonic_time,x,y,z)
        self._vision_speed_lpf = (0.0, 0.0, 0.0)
        self._last_vision_speed_sent = None  # (monotonic_time,vx,vy,vz)
        self._native_flow_quality = 0
        self._native_flow_last_quality_time = 0.0
        self._native_flow_last_stamp = None
        self._native_flow_last_range_m = None
        self._native_flow_last_range_time = 0.0
        self._native_flow_last_range_stamp = None
        self._native_flow_start_time = time.monotonic()
        self._latest_body_rates = None  # monotonic time and roll/pitch/yaw rates
        self._latest_local_pos = None  # (monotonic_time,x,y,z)
        self._latest_global_origin = None  # (monotonic_time, lat, lon, alt_m)
        self._latest_home = None  # (monotonic_time, lat, lon, alt_m)
        self._latest_attitude_rad = None  # (monotonic_time,roll,pitch,yaw)
        self._last_external_nav_diag = 0.0
        self._last_origin_request = 0.0
        self._last_home_request = 0.0
        self._hb_count = 0             # 心跳计数器 (用于周期性调试输出)
        self._last_msg_time = time.monotonic()  # 运行时重连检测
        self._reconnecting = False     # 重连进行中标志

        self._origin_set = False

        # ========== 数据读取定时器 (主线程, 避免锁竞争) ==========
        self.read_timer = self.create_timer(max(self.mavlink_read_period, 0.002), self._read_tick)

        # ========== EKF origin 确认/重试定时器 ==========
        self._origin_timer = self.create_timer(
            max(self.ekf_origin_retry_period, 0.5),
            self._origin_tick,
        )

        # ========== 心跳发布定时器 ==========
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # ========== 解锁状态机定时器 (20Hz, 主线程驱动, 无竞争) ==========
        self.arm_timer = self.create_timer(0.05, self._arm_state_machine)

        # ========== 连接健康监控 (1Hz, 运行时重连) ==========
        self._health_timer = self.create_timer(1.0, self._health_check)

        self.get_logger().info("AP MAVLink 桥节点初始化完成")

    def _open_mavlink_connection(self):
        master = mavutil.mavlink_connection(
            self.connection_url,
            source_system=self.source_system,
            source_component=self.source_component,
        )
        # Disable pymavlink EOF reconnect; this node closes and retries cleanly.
        master.handle_eof = lambda: None
        return master

    def _close_master(self):
        master = getattr(self, "master", None)
        if master is None:
            return
        try:
            master.close()
        except Exception:
            pass
        self.master = None

    def _close_gcs(self):
        link = getattr(self, "gcs_link", None)
        if link is None:
            return
        try:
            link.close()
        except Exception:
            pass
        self.gcs_link = None

    def _mavlink_ready(self) -> bool:
        return self.master is not None and not self._reconnecting

    # ---------- 数据读取 (主线程定时器, 50Hz, 无锁竞争) ----------
    def _read_tick(self):
        """定时器回调：排空所有MAVLink消息 (避免缓冲区堆积导致关键消息延迟)"""
        if not self._mavlink_ready():
            return
        try:
            if self.gcs_link is not None and self.gcs_forward_commands:
                for _ in range(50):
                    gcs_msg = self.gcs_link.recv_match(blocking=False)
                    if gcs_msg is None:
                        break
                    if gcs_msg.get_type() == "BAD_DATA":
                        continue
                    self.master.mav.send(gcs_msg)

            for _ in range(100):  # 最多100条/次, 防止死循环
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    return

                self._last_msg_time = time.monotonic()
                if self.gcs_link is not None and msg.get_type() != "BAD_DATA":
                    self.gcs_link.mav.send(msg)
                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    self._handle_heartbeat(msg)

                elif msg_type == "GLOBAL_POSITION_INT":
                    self.relative_alt = msg.relative_alt / 1000.0  # mm → m
                    self.absolute_alt_msl = msg.alt / 1000.0  # mm → m
                    self._relative_alt_time = time.monotonic()

                elif msg_type == "GPS_GLOBAL_ORIGIN":
                    self._latest_global_origin = (
                        time.monotonic(),
                        msg.latitude / 1e7,
                        msg.longitude / 1e7,
                        msg.altitude / 1000.0,
                    )
                    self._origin_set = True

                elif msg_type == "HOME_POSITION":
                    self._latest_home = (
                        time.monotonic(),
                        msg.latitude / 1e7,
                        msg.longitude / 1e7,
                        msg.altitude / 1000.0,
                    )
                    self.home_ready_pub.publish(Bool(data=True))

                elif msg_type == 'EXTENDED_SYS_STATE':
                    self.landed_pub.publish(Bool(
                        data=msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND))

                elif msg_type == "SYS_STATUS":
                    self.battery_voltage = msg.voltage_battery / 1000.0  # mV → V
                    # EKF_STATUS_REPORT不总是发送，用SYS_STATUS AHRS健康位兜底
                    if not self.ekf_ok:
                        sensor_health = msg.onboard_control_sensors_health
                        # MAV_SYS_STATUS_AHRS = 0x200000 (bit 21)
                        if sensor_health & 0x200000:
                            self.ekf_ok = True

                elif msg_type == "LOCAL_POSITION_NED":
                    self._latest_local_pos = (time.monotonic(), msg.x, msg.y, msg.z)
                    self.local_pos_pub.publish(Point(x=msg.x, y=msg.y, z=msg.z))
                    self.local_velocity_pub.publish(
                        Twist(
                            linear=Vector3(x=msg.vx, y=msg.vy, z=msg.vz),
                        )
                    )

                elif msg_type == "SERVO_OUTPUT_RAW":
                    pwm = [
                        float(msg.servo1_raw),
                        float(msg.servo2_raw),
                        float(msg.servo3_raw),
                        float(msg.servo4_raw),
                    ]
                    self.motor_pwm_pub.publish(Float32MultiArray(data=pwm))

                elif msg_type == "ATTITUDE":
                    self.roll = wrap_angle_rad(float(msg.roll))
                    self.pitch = wrap_angle_rad(float(msg.pitch))
                    self.yaw = wrap_angle_rad(float(msg.yaw))
                    self._latest_attitude_rad = (
                        time.monotonic(),
                        self.roll,
                        self.pitch,
                        self.yaw,
                    )
                    self._latest_body_rates = (
                        time.monotonic(),
                        float(msg.rollspeed),
                        float(msg.pitchspeed),
                        float(msg.yawspeed),
                    )
                    self.rpy_pub.publish(Vector3(x=self.roll, y=self.pitch, z=self.yaw))

                elif msg_type == "EKF_STATUS_REPORT":
                    flags = msg.flags
                    self.ekf_ok = (
                        (flags & (
                            mavutil.mavlink.EKF_POS_HORIZ_ABS
                            | mavutil.mavlink.EKF_POS_HORIZ_REL
                        )) != 0
                        and (flags & (
                            mavutil.mavlink.EKF_POS_VERT_ABS
                            | mavutil.mavlink.EKF_POS_VERT_AGL
                        )) != 0
                    )

                elif msg_type == "COMMAND_ACK":
                    cmd = msg.command
                    result = msg.result
                    # 只记录SET_MODE和NAV_LAND的ACK，减少噪音
                    if cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.get_logger().info("SET_MODE ACK: 已接受")
                        else:
                            self.get_logger().error(
                                f"SET_MODE ACK: 被拒绝 result={result}"
                            )
                    elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.get_logger().info("LAND ACK: 已接受")
                        else:
                            self.get_logger().error(
                                f"LAND ACK: 被拒绝 result={result}"
                            )
                    elif result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.get_logger().warn(f"CMD_ACK cmd={cmd} result={result}")

                elif msg_type == "STATUSTEXT":
                    text = msg.text
                    severity = msg.severity
                    severity_names = {
                        mavutil.mavlink.MAV_SEVERITY_EMERGENCY: "EMERGENCY",
                        mavutil.mavlink.MAV_SEVERITY_ALERT: "ALERT",
                        mavutil.mavlink.MAV_SEVERITY_CRITICAL: "CRITICAL",
                        mavutil.mavlink.MAV_SEVERITY_ERROR: "ERROR",
                    }
                    if severity in severity_names:
                        self.get_logger().warn(f"飞控[{severity_names[severity]}]: {text}")

        except Exception as e:
            self.get_logger().error(f"读取MAVLink消息异常: {e}")

    def _handle_heartbeat(self, msg):
        """处理HEARTBEAT消息"""
        if not self._is_autopilot_heartbeat(msg):
            return
        # mode_string_v10 返回的键与 mode_mapping() 一致 (如 "LOITER", "ALT_HOLD")
        self.current_mode = mavutil.mode_string_v10(msg)
        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @staticmethod
    def _is_autopilot_heartbeat(msg) -> bool:
        if msg.get_type() != "HEARTBEAT":
            return False
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return False
        return msg.type not in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        )

    def request_telemetry(self):
        """请求飞控按指定频率发送遥测数据流
        使用 REQUEST_DATA_STREAM 而非 MAV_CMD_SET_MESSAGE_INTERVAL，
        因为后者在SITL中会触发参数重载(Loaded defaults)导致连接不稳定。
        """
        # MAV_DATA_STREAM 映射: 0=RAW_SENSORS, 1=RC_CHANNELS, 2=RAW_CONTROLLER,
        #   3=EXTENDED_STATUS, 6=EXTRA1(attitude), 10=EXTRA3(global_position),
        #   12=POSITION(local_ned)
        streams = (
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),     # ATTITUDE
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 10),     # GLOBAL_POSITION_INT
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),   # LOCAL_POSITION_NED
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),   # SYS_STATUS
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 15),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, self.raw_controller_hz),  # SERVO_OUTPUT_RAW
        )
        for stream_id, hz in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                hz,
                1,  # start/stop: 1=start
            )
        self._request_message_interval("GPS_GLOBAL_ORIGIN", 1.0)
        self._request_message_interval("HOME_POSITION", 1.0)
        self._request_message_interval('EXTENDED_SYS_STATE', 2.0)
        self.get_logger().info("已请求飞控遥测数据流 (REQUEST_DATA_STREAM)")

    def _request_message_interval(self, msg_name: str, hz: float) -> None:
        if not self._mavlink_ready():
            return
        msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + msg_name, None)
        if msg_id is None:
            return
        interval_us = int(1e6 / hz) if hz > 0 else -1
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"请求{msg_name}消息间隔失败: {exc}",
                throttle_duration_sec=5.0,
            )

    def _request_home_position(self) -> None:
        if not self._mavlink_ready():
            return
        cmd = getattr(mavutil.mavlink, "MAV_CMD_GET_HOME_POSITION", None)
        if cmd is None:
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                cmd,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"请求HOME_POSITION失败: {exc}",
                throttle_duration_sec=5.0,
            )

    def _wait_heartbeat(self, timeout=30):
        """
        等待飞控HEARTBEAT，同时发送心跳推动SITL合成时钟。
        返回HEARTBEAT消息或None（超时）。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not rclpy.ok():
                raise ExternalShutdownException()
            # 发送心跳推动SITL合成时钟
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                pass
            msg = self.master.recv_match(blocking=True, timeout=0.3)
            if msg is None:
                continue
            self._last_msg_time = time.monotonic()
            if msg.get_type() == "HEARTBEAT":
                if not self._is_autopilot_heartbeat(msg):
                    continue
                self.master.target_system = msg.get_srcSystem()
                self.master.target_component = msg.get_srcComponent()
                return msg
        return None

    @staticmethod
    def _decode_param_id(param_id):
        if isinstance(param_id, bytes):
            return param_id.decode("ascii", errors="ignore").strip("\x00")
        return str(param_id).strip("\x00")

    def _wait_param_value(self, name: str, timeout: float = 0.5):
        deadline = time.time() + max(timeout, 0.05)
        while time.time() < deadline:
            remaining = max(0.05, deadline - time.time())
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=remaining)
            if msg is None:
                continue
            if self._decode_param_id(msg.param_id) == name:
                return float(msg.param_value)
        return None

    def _set_param_checked(self, name: str, value: float, mav_type: int, tolerance: float):
        actual = None
        for _attempt in range(2):
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode("ascii"),
                float(value),
                mav_type,
            )
            actual = self._wait_param_value(name, timeout=0.6)
            if actual is None:
                continue
            if abs(actual - float(value)) <= tolerance:
                return actual, True
        return actual, False

    def _setup_critical_params(self):
        """设置关键参数 (避免--defaults触发SITL参数重载风暴)"""
        if not self.simulation_connection:
            return
        if self.localization_backend == "optical_flow_ekf":
            source_params = {
                "EK3_SRC1_POSXY": 0,
                "EK3_SRC1_POSZ": 2,
                "EK3_SRC1_VELXY": 5,
                "EK3_SRC1_VELZ": 0,
                "EK3_SRC1_YAW": 1,
                "EK3_FLOW_USE": 1,
                "FLOW_TYPE": 5,
                "RNGFND1_TYPE": 10,
                "RNGFND1_ORIENT": self.distance_sensor_orientation,
                "VISO_TYPE": 0,
            }
            if not self.enable_native_optical_flow:
                self.get_logger().error(
                    "localization_backend=optical_flow_ekf 但 "
                    "enable_native_optical_flow=false; EKF不会收到光流"
                )
        else:
            velocity_source = 6 if self.send_vision_speed_estimate else 0
            source_params = {
                "EK3_SRC1_POSXY": 6,
                "EK3_SRC1_POSZ": 6,
                "EK3_SRC1_VELXY": velocity_source,
                "EK3_SRC1_VELZ": velocity_source,
                "EK3_SRC1_YAW": 6,
                "EK3_FLOW_USE": 1,
                "VISO_TYPE": 1,
            }
        # 整型参数
        int_params = {
            "FRAME_CLASS": 1,
            "FRAME_TYPE": 1,
            "ARMING_CHECK": 0,
            "BRD_SAFETY_DEFLT": 0,
            "DISARM_DELAY": 0,      # SITL仿真中由任务状态机管理降落/上锁收尾
            "BATT_MONITOR": 0,       # 禁用电池监测(仿真, 避免电压=0触发保护)
            "FS_GCS_ENABLE": 0,      # 禁用失控保护
            "FS_THR_ENABLE": 0,
            "FENCE_ENABLE": 0,
            "GPS_TYPE": 0,
            # Select ExternalNav or ArduPilot's optical-flow/range EKF source.
            **source_params,
            "AHRS_EKF_TYPE": 3,
            "EK3_ENABLE": 1,
            "GUID_OPTIONS": self.ap_guid_options,
        }

        # 浮点参数 (必须用 REAL32 类型, 否则飞控可能拒绝)
        float_params = {
            "VISO_YAW_M_NSE": self.ap_viso_yaw_noise_rad,
            "ARMING_RUDDER": 1.0,
            "SIM_BATT_VOLTAGE": 16.8,  # 仿真电池电压, 避免电压=0触发failsafe
            "MOT_SPIN_MIN": 0.0,
            "MOT_SPIN_ARM": 0.0,
            "MOT_SPIN_MAX": 1.0,
            "MOT_THST_EXPO": 0.0,
            "MOT_THST_HOVER": self.ap_mot_thst_hover,
            "ANGLE_MAX": self.ap_angle_max_cd,
            "PSC_ANGLE_MAX": self.ap_psc_angle_max_deg,
            "ATC_ANG_RLL_P": 3.0,
            "ATC_ANG_PIT_P": 3.0,
            "ATC_ANG_YAW_P": 2.0,
            "ATC_RAT_RLL_P": 0.08,
            "ATC_RAT_RLL_I": 0.04,
            "ATC_RAT_RLL_D": 0.001,
            "ATC_RAT_PIT_P": 0.08,
            "ATC_RAT_PIT_I": 0.04,
            "ATC_RAT_PIT_D": 0.001,
            "ATC_RAT_YAW_P": 0.12,
            "ATC_RAT_YAW_I": 0.02,
            "ATC_RAT_YAW_D": 0.0,
            "ATC_ACCEL_R_MAX": 30000.0,
            "ATC_ACCEL_P_MAX": 30000.0,
            "ATC_ACCEL_Y_MAX": 12000.0,
            "LOIT_SPEED": self.ap_loit_speed_cms,
            "LOIT_ACC_MAX": self.ap_loit_acc_max_cmss,
            "WPNAV_SPEED": self.ap_wpnav_speed_cms,
            "WPNAV_SPEED_UP": self.ap_wpnav_speed_up_cms,
            "WPNAV_SPEED_DN": self.ap_wpnav_speed_dn_cms,
            "WPNAV_RADIUS": 35.0,
            "WPNAV_ACCEL": self.ap_wpnav_accel_cmss,
            "WPNAV_ACCEL_Z": self.ap_wpnav_accel_z_cmss,
            # NAV_TAKEOFF follows the pilot vertical limits rather than the
            # waypoint limits on some ArduCopter builds. Keep both bounded so
            # the first ExternalNav frames cannot be outrun.
            "PILOT_SPEED_UP": self.ap_wpnav_speed_up_cms,
            "PILOT_ACCEL_Z": self.ap_wpnav_accel_z_cmss,
            "PSC_POSXY_P": self.ap_psc_posxy_p,
            "PSC_VELXY_P": self.ap_psc_velxy_p,
            "PSC_VELXY_I": self.ap_psc_velxy_i,
            "PSC_VELXY_D": self.ap_psc_velxy_d,
            "PSC_POSZ_P": 0.45,
            "PSC_VELZ_P": 0.60,
        }
        applied = {}
        failures = []
        for name, value in int_params.items():
            actual, ok = self._set_param_checked(
                name, value, mavutil.mavlink.MAV_PARAM_TYPE_INT32, tolerance=0.5
            )
            applied[name] = actual
            if not ok:
                failures.append((name, value, actual))

        for name, value in float_params.items():
            tol = max(0.02, abs(float(value)) * 0.02)
            actual, ok = self._set_param_checked(
                name, value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, tolerance=tol
            )
            applied[name] = actual
            if not ok:
                failures.append((name, value, actual))

        total = len(int_params) + len(float_params)
        key_names = (
            "FRAME_CLASS", "FRAME_TYPE", "GPS_TYPE",
            "EK3_SRC1_POSXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELXY",
            "EK3_SRC1_VELZ", "EK3_SRC1_YAW", "VISO_TYPE",
            "GUID_OPTIONS", "ANGLE_MAX", "PSC_ANGLE_MAX",
            "ATC_RAT_RLL_P", "ATC_RAT_PIT_P", "ATC_RAT_YAW_P",
            "WPNAV_SPEED", "WPNAV_ACCEL", "MOT_THST_HOVER",
        )
        snapshot = " ".join(
            f"{name}={applied[name]:.3g}" for name in key_names if applied.get(name) is not None
        )
        if snapshot:
            self.get_logger().info(f"关键参数回读: {snapshot}")
        if failures:
            detail = ", ".join(
                f"{name}:期望{expected}/实际{actual if actual is not None else '未回读'}"
                for name, expected, actual in failures
            )
            self.get_logger().warn(
                f"关键参数部分未通过回读校验 ({len(failures)}/{total}): {detail}"
            )
        else:
            self.get_logger().info(f"关键参数已设置并回读通过 ({total}项, 含无GPS导航)")

    # ---------- 定时发布 ----------
    def publish_heartbeat(self):
        """1Hz 发布心跳和状态"""
        self._hb_count += 1

        # M1修复: 重连期间发布connected=False, 避免1秒窗口stale connected=True
        if not self._mavlink_ready():
            self.connected_pub.publish(Bool(data=False))
            return

        # 主动发送MAVLink心跳推动SITL合成时钟。
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
        except Exception:
            pass

        # 连接状态
        connected_msg = Bool(data=True)
        self.connected_pub.publish(connected_msg)

        # 飞行模式
        mode_msg = String(data=self.current_mode)
        self.mode_pub.publish(mode_msg)

        # 解锁状态
        armed_msg = Bool(data=self.is_armed)
        self.armed_pub.publish(armed_msg)

        # 相对高度
        alt_msg = Float32(data=self.relative_alt)
        self.alt_pub.publish(alt_msg)

        # 电池电压
        batt_msg = Float32(data=self.battery_voltage)
        self.batt_pub.publish(batt_msg)

        # 姿态角 — 已在 read_loop 中高频发布 (ATTITUDE ~10Hz)
        # 此处不再重复发布, 避免与 read_loop 线程竞争

        # EKF状态
        ekf_msg = Bool(data=self.ekf_ok)
        self.ekf_ok_pub.publish(ekf_msg)

        # 每5秒打印一次状态快照, 方便诊断模式切换/高度/解锁状态
        if self._hb_count % 5 == 0:
            self.get_logger().info(
                f"[状态] 模式={self.current_mode} 解锁={self.is_armed} "
                f"高度={self.relative_alt:.2f}m 电压={self.battery_voltage:.1f}V "
                f"EKF={self.ekf_ok}"
            )

    # ---------- 控制回调 ----------
    def _resolve_mode(self, mode_name: str):
        """解析模式名 -> mode_id, 支持常见变体 (大小写/下划线差异)"""
        mode_map = self.master.mode_mapping()
        # 1. 精确匹配
        if mode_name in mode_map:
            return mode_map[mode_name]
        # 2. 大写 + 去下划线匹配
        upper_nous = mode_name.strip().upper().replace("_", "")
        for key, val in mode_map.items():
            if key.strip().upper().replace("_", "") == upper_nous:
                self.get_logger().info(f"模式名规范化: '{mode_name}' -> '{key}'")
                return val
        # 3. 大写 + 加下划线匹配 (如 ALTHOLD -> ALT_HOLD)
        for key, val in mode_map.items():
            if key.strip().upper() == mode_name.strip().upper():
                self.get_logger().info(f"模式名规范化: '{mode_name}' -> '{key}'")
                return val
        return None

    def set_mode_cb(self, msg: String):
        """切换飞行模式 (使用 MAV_CMD_DO_SET_MODE + command_long_send)

        C4修复: 不再阻塞等待COMMAND_ACK。
        原来的 blocking=True, timeout=3.0 会在ROS订阅回调中阻塞执行器，
        导致 arm_cb/_read_tick/_arm_state_machine 全部饿死。
        ACK现在由 _read_tick 非阻塞接收并记录。
        """
        mode_name = msg.data.strip().upper()

        mode_id = self._resolve_mode(mode_name)
        if mode_id is None:
            available = list(self.master.mode_mapping().keys())
            self.get_logger().error(f"未知模式 '{mode_name}', 可用: {available}")
            return

        self.get_logger().info(
            f"发送模式切换: '{mode_name}' -> mode_id={mode_id} "
            f"(当前模式={self.current_mode})"
        )

        # 使用 command_long_send + MAV_CMD_DO_SET_MODE
        # 关键: target_component 必须为 MAV_COMP_ID_AUTOPILOT1 (1),
        # 因为飞控的 autopilot 组件是 1，0 会广播到所有组件但不保证被飞控处理。
        self.master.mav.command_long_send(
            self.master.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,   # 目标组件=飞控autopilot
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0,  # param3-7 unused
        )

        self.get_logger().info(
            f"模式切换指令已发送(非阻塞): '{mode_name}', HEARTBEAT将确认模式切换"
        )

    def arm_cb(self, msg: Bool):
        """解锁/上锁 — 发送MAVLink ARM命令 + RC中立值维持"""
        if msg.data:
            if self.is_armed:
                return
            ts = self.master.target_system
            tc = self.master.target_component
            # 持续发送RC中立值 (防止全0触发failsafe)
            self.master.mav.rc_channels_override_send(
                ts, tc, 1500, 1500, 1100, 1500, 0, 0, 0, 0)
            # MAV_CMD_COMPONENT_ARM_DISARM 直接解锁
            self.master.mav.command_long_send(
                ts, tc,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            self.get_logger().info("已发送 ARM 指令 (MAV_CMD)")
        else:
            # 重置RC覆盖 (否则飞控持续锁定)
            ts = self.master.target_system
            tc = self.master.target_component
            self.master.mav.rc_channels_override_send(
                ts, tc, 0, 0, 0, 0, 0, 0, 0, 0)
            if self.force_disarm:
                self.master.mav.command_long_send(
                    ts, tc,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,        # disarm
                    21196,    # force disarm magic, simulation only by default
                    0, 0, 0, 0, 0,
                )
                self.get_logger().warn("已发送 FORCE DISARM 指令")
            else:
                self.master.arducopter_disarm()
                self.get_logger().info("已发送 DISARM 指令")

    def _arm_state_machine(self):
        """主线程定时器驱动 (20Hz), 持续发送RC摇杆解锁信号直到成功或超时"""
        sm = self._arm_sm
        if sm == "idle":
            return

        now = time.monotonic()
        if now < self._arm_sm_deadline:
            return

        ts = self.master.target_system
        tc = self.master.target_component
        elapsed = now - self._arm_start_time

        if sm == "arming":
            if self.is_armed:
                # 解锁成功! 维持RC中立值 (释放全0会触发failsafe自动上锁)
                self.master.mav.rc_channels_override_send(
                    ts, tc, 1500, 1500, 1100, 1500, 0, 0, 0, 0)
                self.get_logger().info(f"ARM 成功! (RC摇杆, 耗时{elapsed:.1f}s) 维持中立RC")
                self._arm_sm = "idle"
                return

            if elapsed > 20.0:
                self.master.mav.rc_channels_override_send(
                    ts, tc, 0, 0, 0, 0, 0, 0, 0, 0)
                self.get_logger().error(f"RC摇杆解锁超时({elapsed:.0f}s)! 请检查飞控状态")
                self._arm_sm = "idle"
                return

            # 持续发送油门最低(1100) + 方向最右(1900)
            self.master.mav.rc_channels_override_send(
                ts, tc, 1500, 1500, 1100, 1900, 0, 0, 0, 0)
            if elapsed < 0.5:
                self.get_logger().info("RC摇杆解锁: 油门低+方向右 (持续发送直到解锁)...")
            self._arm_sm_deadline = now + 0.5  # 每0.5秒发送一次

    def land_cb(self, msg: Bool):
        """降落"""
        if not msg.data:
            return

        if not self.set_ekf_origin_on_start and self._latest_home is None:
            self.get_logger().warn(
                "无GPS原点/HOME_POSITION, 拒绝切换LAND以避免ArduPilot invalid location panic"
            )
            return

        land_id = self._resolve_mode("LAND")
        if land_id is not None:
            self.master.mav.command_long_send(
                self.master.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                land_id,
                0, 0, 0, 0, 0,
            )
            self.get_logger().info("已请求 LAND 模式")
            return

        self.get_logger().warn("LAND模式不可用, 未发送MAV_CMD_NAV_LAND以避免无GPS原点panic")

    def _health_check(self):
        """1Hz健康监控: 检测飞控消息超时, 触发运行时重连"""
        now = time.monotonic()

        # 正在重连中: 每次tick尝试一次重连, 直到成功
        if self._reconnecting:
            try:
                self._close_master()
                self.master = self._open_mavlink_connection()

                hb = self._wait_heartbeat(timeout=10)
                if hb is None:
                    raise TimeoutError("HEARTBEAT超时 (10s)")

                self._handle_heartbeat(hb)
                self._reconnecting = False
                self._last_msg_time = now
                self.request_telemetry()
                self.connected_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f"运行时重连成功! target={self.master.target_system}"
                )
            except Exception as e:
                self._close_master()
                self.get_logger().warn(f"运行时重连尝试失败: {e}", throttle_duration_sec=5.0)
            return

        # 消息超时检测: 3秒无消息视为断连
        if now - self._last_msg_time < 3.0:
            return

        self.get_logger().error(
            f"飞控消息超时 {now - self._last_msg_time:.1f}s, 触发运行时重连..."
        )
        self._reconnecting = True
        self.connected_pub.publish(Bool(data=False))
        self._close_master()

    def takeoff_cb(self, msg: Float32):
        """GUIDED takeoff to the requested relative altitude in meters."""
        target_alt = float(msg.data)
        if not math.isfinite(target_alt):
            self.get_logger().warn("GUIDED起飞高度NaN/Inf, 忽略", throttle_duration_sec=1.0)
            return
        target_alt = max(0.1, target_alt)
        if (
            target_alt < self.nav_takeoff_min_altitude
            and not self.allow_low_alt_nav_takeoff
        ):
            self.get_logger().error(
                f"拒绝低空NAV_TAKEOFF: target_relative_alt={target_alt:.2f}m "
                f"< min={self.nav_takeoff_min_altitude:.2f}m"
            )
            return

        # ArduCopter handles NAV_TAKEOFF through COMMAND_INT and requires
        # MAV_FRAME_GLOBAL_RELATIVE_ALT. Lat/lon are ignored by Copter here.
        self.master.mav.command_int_send(
            self.master.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            0,
            target_alt,
        )
        self.get_logger().info(
            f"已发送GUIDED起飞命令: target_relative_alt={target_alt:.2f}m "
            f"current_rel={self.relative_alt:.2f}m"
        )

    def release_rc_override_cb(self, msg: Bool):
        """释放 RC_CHANNEL_OVERRIDE, 让 GUIDED/LOITER 自主控制接管。"""
        if not msg.data:
            return
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        self.get_logger().info("已释放 RC override (所有通道=0)")

    def rc_override_cb(self, msg: Joy):
        """
        RC通道覆盖 - 近距视觉伺服、调试和应急控制接口
        Joy.axes[0]=ch1(roll), [1]=ch2(pitch), [2]=ch3(throttle), [3]=ch4(yaw)
        支持两种输入格式:
          - PWM范围: 1100-1900 (中立=1500)
          - 归一化范围: [-1.0, 1.0] → 自动缩放到 [1100, 1900]
        """
        # 读取原始值
        raw = [
            msg.axes[0] if len(msg.axes) > 0 else 0.0,
            msg.axes[1] if len(msg.axes) > 1 else 0.0,
            msg.axes[2] if len(msg.axes) > 2 else 0.0,
            msg.axes[3] if len(msg.axes) > 3 else 0.0,
        ]

        # 检测范围: 如果所有非零值都在 [-2, 2] 区间, 判定为归一化输入
        nonzero = [v for v in raw if abs(v) > 0.001]
        is_normalized = nonzero and all(-2.0 <= v <= 2.0 for v in nonzero)

        pwm = []
        for i, val in enumerate(raw):
            if is_normalized:
                # 归一化 [-1, 1] -> PWM [1100, 1900], 中立点 0 -> 1500
                scaled = int(1500 + val * 400)
            else:
                scaled = int(val) if abs(val) > 0.001 else 1500
            # 夹紧到有效 PWM 范围
            pwm.append(max(1100, min(1900, scaled)))

        ch1, ch2, ch3, ch4 = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            ch1, ch2, ch3, ch4,
            0, 0, 0, 0,  # ch5-ch8 设为0(不覆盖)
        )

        # 每5秒打印一次调试信息
        now = time.monotonic()
        last = getattr(self, "_rc_last_log", 0.0)
        if now - last >= 5.0:
            self._rc_last_log = now
            scaled_note = " (归一化→PWM)" if is_normalized else ""
            self.get_logger().info(
                f"[RC超控] ch1={ch1} ch2={ch2} ch3={ch3}(油门) ch4={ch4} "
                f"模式={self.current_mode} 解锁={self.is_armed} 高度={self.relative_alt:.2f}m"
                f"{scaled_note}"
            )

    def _send_position_target_local_ned(
        self,
        frame,
        type_mask,
        x=0.0,
        y=0.0,
        z=0.0,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw=0.0,
        yaw_rate=0.0,
    ):
        """发送 SET_POSITION_TARGET_LOCAL_NED。"""
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            frame,
            type_mask,
            x, y, z,
            vx, vy, vz,
            0, 0, 0,
            yaw,
            yaw_rate,
        )

    def guided_position_cb(self, msg: Point):
        """
        GUIDED位置目标: /ap/cmd/guided_position
        Point 使用 FCU local NED: x=North, y=East, z=Down, 单位米。
        """
        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.get_logger().warn("GUIDED位置目标NaN/Inf, 忽略", throttle_duration_sec=1.0)
            return

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
        yaw = self.guided_yaw_lock_rad
        if self.guided_yaw_lock_enabled:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        else:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        send_y = -y if self.invert_guided_position_y else y
        self._send_position_target_local_ned(
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x=x,
            y=send_y,
            z=z,
            yaw=yaw,
        )

        now = time.monotonic()
        last = getattr(self, "_guided_pos_last_log", 0.0)
        if now - last >= 3.0:
            self._guided_pos_last_log = now
            yaw_note = f"{math.degrees(yaw):.1f}deg" if self.guided_yaw_lock_enabled else "ignore"
            inv_note = " invY" if self.invert_guided_position_y else ""
            self.get_logger().info(
                f"[GUIDED位置] target_ned=({x:.2f},{y:.2f},{z:.2f}){inv_note} "
                f"yaw={yaw_note} yaw_rate=ignore "
                f"模式={self.current_mode} 解锁={self.is_armed}"
            )

    def guided_velocity_cb(self, msg: Twist):
        """
        GUIDED局部速度目标: /ap/cmd/guided_velocity
        Twist.linear 使用 FCU local NED 速度, angular.z 为 yaw_rate(rad/s)。
        """
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)
        yaw_rate = float(msg.angular.z)
        if not all(math.isfinite(v) for v in (vx, vy, vz, yaw_rate)):
            self.get_logger().warn("GUIDED速度目标NaN/Inf, 忽略", throttle_duration_sec=1.0)
            return

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
        yaw = self.guided_yaw_lock_rad
        if self.guided_yaw_lock_enabled:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_rate = 0.0
        else:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        send_vy = -vy if self.invert_guided_velocity_y else vy
        self._send_position_target_local_ned(
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            vx=vx,
            vy=send_vy,
            vz=vz,
            yaw=yaw,
            yaw_rate=yaw_rate,
        )

    def guided_level_climb_cb(self, msg: Float32):
        """
        GUIDED水平姿态 + 爬升率目标。
        用于受限离地后的交权稳定段, 不承担航点导航。
        """
        climb_rate = float(msg.data)
        if not math.isfinite(climb_rate):
            self.get_logger().warn("GUIDED水平爬升率NaN/Inf, 忽略", throttle_duration_sec=1.0)
            return

        full_scale = max(abs(self.guided_attitude_climb_full_scale), 0.1)
        climb_rate = max(-full_scale, min(full_scale, climb_rate))
        raw_thrust = self.guided_attitude_hover_thrust + climb_rate / (2.0 * full_scale)
        thrust = max(
            self.guided_attitude_thrust_min,
            min(self.guided_attitude_thrust_max, raw_thrust),
        )
        yaw = wrap_angle_rad(self.yaw)
        q = euler_to_quaternion(0.0, 0.0, yaw)
        type_mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        )
        self.master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            type_mask,
            q,
            0.0,
            0.0,
            0.0,
            thrust,
        )

        now = time.monotonic()
        last = getattr(self, "_guided_att_last_log", 0.0)
        if now - last >= 2.0:
            self._guided_att_last_log = now
            self.get_logger().info(
                f"[GUIDED水平爬升] climb_rate={climb_rate:.2f}m/s "
                f"thrust_field={thrust:.3f} raw={raw_thrust:.3f} "
                f"hover={self.guided_attitude_hover_thrust:.3f} "
                f"yaw_hold={math.degrees(yaw):.1f}deg "
                f"模式={self.current_mode} 解锁={self.is_armed}"
            )

    def body_velocity_cb(self, msg: Twist):
        """
        机体速度控制 - 备用方式
        vx,vy,vz 单位 m/s, angular.z 单位 rad/s
        """
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)
        yaw_rate = float(msg.angular.z)

        # 使用 POSITION_TARGET 消息 (忽略位置, 只控制速度)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self._send_position_target_local_ned(
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
        )

    def native_optical_flow_quality_cb(self, msg: UInt8):
        """Cache the camera quality byte used by ``OPTICAL_FLOW_RAD``."""
        self._native_flow_quality = max(0, min(255, int(msg.data)))
        self._native_flow_last_quality_time = time.monotonic()

    @staticmethod
    def _ros_stamp_seconds(stamp):
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if value > 0.0 else time.monotonic()

    def native_distance_sensor_cb(self, msg: Range):
        """Forward a valid downward range sample in centimetres."""
        if not self._mavlink_ready():
            return
        distance = float(msg.range)
        if not math.isfinite(distance):
            return
        minimum = max(self.distance_sensor_min_m, float(msg.min_range))
        maximum = min(self.distance_sensor_max_m, float(msg.max_range))
        if maximum < minimum:
            minimum, maximum = self.distance_sensor_min_m, self.distance_sensor_max_m
        if distance < minimum or distance > maximum:
            return
        self._native_flow_last_range_m = distance
        self._native_flow_last_range_time = time.monotonic()
        self._native_flow_last_range_stamp = self._ros_stamp_seconds(msg.header.stamp)
        self.master.mav.distance_sensor_send(
            int((time.monotonic() - self._native_flow_start_time) * 1000.0)
            & 0xFFFFFFFF,
            int(round(minimum * 100.0)),
            int(round(maximum * 100.0)),
            int(round(distance * 100.0)),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            self.distance_sensor_id,
            self.distance_sensor_orientation,
            0,
        )

    def native_optical_flow_cb(self, msg: TwistStamped):
        """Convert body-FLU velocity to integrated angular optical flow.

        ArduPilot expects radians integrated over the sample interval.  For a
        down-looking sensor, ``flow_x ~= -v_right / height`` and
        ``flow_y ~= v_forward / height``; the FLU-to-FRD conversion makes the
        default signs below equivalent to that convention.  Installation
        signs remain parameters because camera drivers differ in axis order.
        """
        if not self._mavlink_ready():
            return
        now = time.monotonic()
        if now - self._native_flow_last_quality_time > self.optical_flow_max_dt:
            return
        quality = self._native_flow_quality
        if quality < self.optical_flow_min_quality:
            return
        height = self._native_flow_last_range_m
        range_stamp = self._native_flow_last_range_stamp
        if height is None or range_stamp is None or not math.isfinite(height):
            return
        if now - self._native_flow_last_range_time > self.optical_flow_max_dt:
            return
        if not self.optical_flow_min_height <= height <= self.optical_flow_max_height:
            return

        stamp = self._ros_stamp_seconds(msg.header.stamp)
        if self._native_flow_last_stamp is None:
            self._native_flow_last_stamp = stamp
            return
        dt = stamp - self._native_flow_last_stamp
        self._native_flow_last_stamp = stamp
        if not self.optical_flow_min_dt <= dt <= self.optical_flow_max_dt:
            return

        vx_flu = float(msg.twist.linear.x)
        vy_flu = float(msg.twist.linear.y)
        if not all(math.isfinite(value) for value in (vx_flu, vy_flu)):
            return
        if self._latest_body_rates is None:
            return
        gyro_time, roll_rate, pitch_rate, _yaw_rate = self._latest_body_rates
        if now - gyro_time > self.optical_flow_max_dt:
            return
        # FLU (left) to FRD (right), then translational image motion.
        flow_rates = velocity_to_flow_rates(
            vx_flu,
            vy_flu,
            height,
            sensor_yaw_rad=self.optical_flow_sensor_yaw_rad,
            x_sign=self.optical_flow_x_sign,
            y_sign=self.optical_flow_y_sign,
            max_rate_rad_s=self.optical_flow_max_rate_rad_s,
        )
        if flow_rates is None:
            return
        # The input velocity is translation-only. Reconstruct raw angular flow
        # because ArduPilot applies body-rate compensation with its own IMU.
        cosine = math.cos(self.optical_flow_sensor_yaw_rad)
        sine = math.sin(self.optical_flow_sensor_yaw_rad)
        sensor_roll_rate = cosine * roll_rate + sine * pitch_rate
        sensor_pitch_rate = -sine * roll_rate + cosine * pitch_rate
        flow_x_rate = flow_rates[0] + sensor_roll_rate
        flow_y_rate = flow_rates[1] + sensor_pitch_rate
        if max(abs(flow_x_rate), abs(flow_y_rate)) > self.optical_flow_max_rate_rad_s:
            return

        time_usec = int(stamp * 1.0e6) & 0xFFFFFFFFFFFFFFFF
        integration_time_us = int(round(dt * 1.0e6))
        range_age_us = int(round(max(0.0, stamp - range_stamp) * 1.0e6))
        if range_age_us > int(self.optical_flow_max_dt * 1.0e6):
            return
        self.master.mav.optical_flow_rad_send(
            time_usec,
            self.optical_flow_sensor_id,
            integration_time_us,
            flow_x_rate * dt,
            flow_y_rate * dt,
            0.0,
            0.0,
            0.0,
            0,
            quality,
            range_age_us,
            float(height),
        )

    def vision_pose_cb(self, msg: PoseWithCovarianceStamped):
        """
        ExternalNav位姿回调: /ap/external_nav/vision_pose 采用 FCU local NED 契约。
        这里只解码同一 NED 坐标系下的四元数并转发 VISION_POSITION_ESTIMATE,
        不再做 ENU→NED 二次变换。
        """
        if not self.enable_external_nav:
            return
        if not self._mavlink_ready():
            self.get_logger().warn(
                "MAVLink未连接, 暂停发送ExternalNav位姿",
                throttle_duration_sec=2.0,
            )
            return

        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ned_x = float(pos.x)
        ned_y = float(pos.y)
        ned_z = float(pos.z)
        if (
            self.external_nav_baro_z_fusion
            and time.monotonic() - self._relative_alt_time <= self.external_nav_baro_z_max_age
            and math.isfinite(self.relative_alt)
        ):
            # Keep XY/yaw from the LiDAR registration while using the
            # GPS-independent barometric height for the weakly observable Z
            # component. The message remains LOCAL_NED and is still sent as
            # VISION_POSITION_ESTIMATE; no GPS source is enabled.
            ned_z = -float(self.relative_alt)
        ned_roll, ned_pitch, ned_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        external_nav_yaw = ned_yaw
        yaw_source_used = "external_nav"
        if self.force_vision_flat_attitude:
            ned_roll = 0.0
            ned_pitch = 0.0
        if self.vision_yaw_source == "fcu_attitude":
            now = time.monotonic()
            if self._latest_attitude_rad is None:
                self.get_logger().warn(
                    "vision_yaw_source=fcu_attitude 但尚无飞控姿态, 暂停发送ExternalNav",
                    throttle_duration_sec=1.0,
                )
                return
            att_t, _roll, _pitch, fcu_yaw = self._latest_attitude_rad
            att_age = now - att_t
            if att_age > self.vision_yaw_attitude_max_age:
                self.get_logger().warn(
                    f"飞控姿态过旧({att_age:.2f}s), 暂停发送ExternalNav",
                    throttle_duration_sec=1.0,
                )
                return
            ned_yaw = fcu_yaw
            yaw_source_used = "fcu_attitude"

        if not all(math.isfinite(v) for v in (ned_x, ned_y, ned_z, ned_roll, ned_pitch, ned_yaw)):
            self.get_logger().warn(
                "ExternalNav位姿NaN/Inf! 忽略",
                throttle_duration_sec=1.0,
            )
            return
        ned_x, ned_y, ned_z, pose_guard_note = self._guard_vision_pose_jump(
            ned_x,
            ned_y,
            ned_z,
        )
        self.last_vision_pose = (ned_x, ned_y, ned_z, ned_roll, ned_pitch, ned_yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        sample_time = stamp if stamp > 0. else None
        self._send_vision_speed_if_available(ned_x, ned_y, ned_z, sample_time=sample_time)

        cov_list = list(msg.pose.covariance)
        use_cov_xy = cov_list[0] if len(cov_list) > 0 and cov_list[0] > 0 else self.cov_xy
        use_cov_z = cov_list[14] if len(cov_list) > 14 and cov_list[14] > 0 else self.cov_z
        use_cov_yaw = cov_list[35] if len(cov_list) > 35 and cov_list[35] > 0 else self.cov_yaw
        use_cov_rp = cov_list[28] if len(cov_list) > 28 and cov_list[28] > 0 else self.cov_rp

        cov_upper = [
            use_cov_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            use_cov_xy, 0.0, 0.0, 0.0, 0.0,
            use_cov_z, 0.0, 0.0, 0.0,
            use_cov_rp, 0.0, 0.0,
            use_cov_rp, 0.0,
            use_cov_yaw,
        ]

        usec = int(sample_time * 1e6) if sample_time is not None else int(self.get_clock().now().nanoseconds / 1000)
        self.master.mav.vision_position_estimate_send(
            usec,
            ned_x, ned_y, ned_z,
            ned_roll, ned_pitch, ned_yaw,
            cov_upper,
            reset_counter=0,
        )
        self._last_vision_pose_sent = (time.monotonic(), ned_x, ned_y, ned_z)
        self._log_external_nav_diagnostics(
            ned_x,
            ned_y,
            ned_z,
            ned_yaw,
            yaw_source=yaw_source_used,
            external_nav_yaw=external_nav_yaw,
            pose_guard_note=pose_guard_note,
        )

    def _guard_vision_pose_jump(self, x, y, z):
        if not self.vision_pose_jump_guard_enabled:
            return x, y, z, ""
        now = time.monotonic()
        if self._last_vision_pose_sent is None:
            self._vision_pose_jump_hold_until = 0.0
            return x, y, z, ""

        last_t, last_x, last_y, last_z = self._last_vision_pose_sent
        dt = max(now - last_t, 1e-3)
        jump_xy = math.hypot(x - last_x, y - last_y)
        allowed = max(self.vision_pose_jump_max_m, self.vision_pose_jump_max_mps * dt)
        if jump_xy <= allowed:
            self._vision_pose_jump_hold_until = 0.0
            return x, y, z, ""

        if self._vision_pose_jump_hold_until <= 0.0:
            self._vision_pose_jump_hold_until = now + self.vision_pose_jump_hold_timeout

        if now <= self._vision_pose_jump_hold_until:
            remaining = self._vision_pose_jump_hold_until - now
            self.get_logger().warn(
                f"ExternalNav位置跳变 {jump_xy:.2f}m>{allowed:.2f}m, "
                f"保持上一可信XY {remaining:.1f}s",
                throttle_duration_sec=1.0,
            )
            return last_x, last_y, last_z, f"pose_guard=hold jump_xy={jump_xy:.2f}m"

        self.get_logger().warn(
            f"ExternalNav位置跳变持续 {jump_xy:.2f}m, 超过保持时间后接受新位置",
            throttle_duration_sec=2.0,
        )
        self._vision_pose_jump_hold_until = 0.0
        return x, y, z, f"pose_guard=accepted_jump jump_xy={jump_xy:.2f}m"

    def _log_external_nav_diagnostics(
        self,
        x,
        y,
        z,
        yaw,
        yaw_source="external_nav",
        external_nav_yaw=None,
        pose_guard_note="",
    ):
        """Throttle diagnostic output for ExternalNav -> EKF local alignment."""
        if not self.external_nav_diagnostics:
            return
        now = time.monotonic()
        period = max(self.external_nav_diagnostics_period, 0.2)
        if now - self._last_external_nav_diag < period:
            return
        self._last_external_nav_diag = now

        local_text = "local=NA"
        if self._latest_local_pos is not None:
            local_t, lx, ly, lz = self._latest_local_pos
            age = now - local_t
            xy_err = math.hypot(float(lx) - x, float(ly) - y)
            z_err = float(lz) - z
            local_text = (
                f"local=({lx:.2f},{ly:.2f},{lz:.2f}) "
                f"age={age:.2f}s dxy={xy_err:.2f} dz={z_err:.2f}"
            )

        attitude_text = "att=NA"
        if self._latest_attitude_rad is not None:
            att_t, roll, pitch, fcu_yaw = self._latest_attitude_rad
            yaw_err = math.atan2(math.sin(float(fcu_yaw) - yaw), math.cos(float(fcu_yaw) - yaw))
            attitude_text = (
                f"att_rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},"
                f"{math.degrees(fcu_yaw):.1f}) yaw_err={math.degrees(yaw_err):.1f}deg "
                f"age={now - att_t:.2f}s"
            )

        speed_text = "vext=NA"
        if self._last_vision_speed_sent is not None:
            spd_t, vx, vy, vz = self._last_vision_speed_sent
            speed_text = (
                f"vext=({vx:.2f},{vy:.2f},{vz:.2f}) "
                f"|vxy|={math.hypot(vx, vy):.2f} age={now - spd_t:.2f}s"
            )

        source_text = f"yaw_source={yaw_source}"
        if external_nav_yaw is not None and yaw_source != "external_nav":
            source_text += (
                f" external_nav_yaw={math.degrees(external_nav_yaw):.1f}deg"
            )
        if pose_guard_note:
            source_text += f" {pose_guard_note}"

        self.get_logger().info(
            f"[ExternalNav诊断] vision=({x:.2f},{y:.2f},{z:.2f}) "
            f"yaw={math.degrees(yaw):.1f}deg {source_text} {speed_text} "
            f"{local_text} {attitude_text} mode={self.current_mode} armed={self.is_armed}"
        )

    def _send_vision_speed_if_available(self, x, y, z, sample_time=None):
        """Send VISION_SPEED_ESTIMATE derived from the ExternalNav pose stream."""
        if not self.send_vision_speed_estimate:
            return
        if not self._mavlink_ready():
            return

        now = time.monotonic()
        sample_time = now if sample_time is None else sample_time
        previous = self._last_vision_speed_sample
        self._last_vision_speed_sample = (sample_time, x, y, z)
        if previous is None:
            return

        prev_t, prev_x, prev_y, prev_z = previous
        dt = sample_time - prev_t
        if dt < self.vision_speed_min_dt or dt > self.vision_speed_max_dt:
            self._vision_speed_lpf = (0.0, 0.0, 0.0)
            return

        vx = (x - prev_x) / dt
        vy = (y - prev_y) / dt
        vz = (z - prev_z) / dt
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if not math.isfinite(speed):
            return
        if speed > self.vision_speed_jump_mps:
            self.get_logger().warn(
                f"ExternalNav速度跳变 {speed:.1f}m/s, 本帧不发送VISION_SPEED_ESTIMATE",
                throttle_duration_sec=1.0,
            )
            self._vision_speed_lpf = (0.0, 0.0, 0.0)
            return
        if self.vision_speed_max_mps > 0.0 and speed > self.vision_speed_max_mps:
            scale = self.vision_speed_max_mps / speed
            vx *= scale
            vy *= scale
            vz *= scale

        alpha = max(0.0, min(1.0, self.vision_speed_lpf_alpha))
        old_vx, old_vy, old_vz = self._vision_speed_lpf
        vx = alpha * vx + (1.0 - alpha) * old_vx
        vy = alpha * vy + (1.0 - alpha) * old_vy
        vz = alpha * vz + (1.0 - alpha) * old_vz
        self._vision_speed_lpf = (vx, vy, vz)

        cov = [
            self.cov_vel, 0.0, 0.0,
            0.0, self.cov_vel, 0.0,
            0.0, 0.0, self.cov_vel,
        ]
        usec = int(sample_time * 1e6)
        self.master.mav.vision_speed_estimate_send(
            usec,
            vx,
            vy,
            vz,
            cov,
            reset_counter=0,
        )
        self._last_vision_speed_sent = (now, vx, vy, vz)

    def set_ekf_origin(self):
        """设置EKF原点 — 室内无GPS导航的必要步骤"""
        if not self._mavlink_ready():
            return False
        if self._origin_set:
            return True
        origin_alt_msl = self._current_origin_alt_msl()
        lat_int = int(ORIGIN_LAT * 1e7)
        lon_int = int(ORIGIN_LON * 1e7)
        alt_mm = int(origin_alt_msl * 1000)

        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            lat_int, lon_int, alt_mm,
            0,  # time_usec
        )
        self._last_origin_request = time.monotonic()
        self.get_logger().info(
            f"已发送EKF原点设置请求: lat={ORIGIN_LAT:.6f} lon={ORIGIN_LON:.6f} "
            f"alt={origin_alt_msl:.2f}m (等待飞控确认)"
        )
        self._request_message_interval("GPS_GLOBAL_ORIGIN", 1.0)
        self._request_message_interval("HOME_POSITION", 1.0)
        self._request_home_position()
        return False

    def _origin_tick(self):
        """确认或重试室内EKF global origin。"""
        if not (self.enable_external_nav and self.set_ekf_origin_on_start):
            return
        if not self._mavlink_ready():
            return

        now = time.monotonic()
        self.home_ready_pub.publish(Bool(data=self._latest_home is not None))
        if (self.set_home_on_start and self._latest_home is None and not self.is_armed
                and now - self._last_origin_request >= max(self.ekf_origin_retry_period, .5)):
            # An early origin acknowledgement may precede EKF alignment.
            # Repeating SET_GPS_GLOBAL_ORIGIN is idempotent in ArduPilot once
            # the real EKF origin exists; stop all initialization at HOME ack.
            self._origin_set = False
            self.set_ekf_origin()
        if self._latest_global_origin is not None and (
            not self.set_home_on_start or self._latest_home is not None
            or now - self._latest_global_origin[0] < 5.
        ):
            origin_t, lat, lon, alt = self._latest_global_origin
            if not self._origin_set:
                self.get_logger().info(f'EKF原点已确认: lat={lat:.6f} lon={lon:.6f} alt={alt:.2f}m')
            self._origin_set = True
            if (self.set_home_on_start and self._latest_home is None and not self.is_armed
                    and now - self._last_home_request > 2.):
                # HOME is a coordinate reference, not a GPS observation.
                self.master.mav.command_int_send(
                    self.master.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                    0, 0, 0., 0., 0., 0., int(lat * 1e7), int(lon * 1e7), float(alt),
                )
                self._last_home_request = now
                self.get_logger().info(f'已请求设置HOME参考: lat={lat:.6f} lon={lon:.6f} alt={alt:.2f}m')
                self._request_home_position()
            return

        # EKF startup/reset can discard an origin acknowledged during early
        # initialization. Retry until HOME is confirmed instead of trusting
        # that old one-shot acknowledgement forever.
        if self.set_home_on_start and self._latest_home is None:
            self._origin_set = False
            self._latest_global_origin = None

        self._request_message_interval("GPS_GLOBAL_ORIGIN", 1.0)
        self._request_message_interval("HOME_POSITION", 1.0)
        self._request_home_position()

        if now - self._last_origin_request < max(self.ekf_origin_retry_period, 0.5):
            return
        if self._latest_local_pos is not None:
            local_t, lx, ly, lz = self._latest_local_pos
            if now - local_t <= 2.0:
                self.get_logger().warn(
                    "LOCAL_POSITION_NED已输出, 但GPS_GLOBAL_ORIGIN/HOME_POSITION仍缺失; "
                    f"继续重发EKF origin。local=({lx:.2f},{ly:.2f},{lz:.2f})",
                    throttle_duration_sec=10.0,
                )
        self.set_ekf_origin()

    def _current_origin_alt_msl(self, timeout_s: float = 2.0) -> float:
        """Use current FCU ASL altitude for EKF origin height.

        A fixed origin altitude biases GLOBAL_POSITION_INT.relative_alt and
        LOCAL_POSITION_NED.z when the barometer estimate drifts from that
        constant. Reading the current FCU altitude keeps the ground-relative
        height near zero at stack startup.
        """
        if self.absolute_alt_msl is not None and math.isfinite(self.absolute_alt_msl):
            return float(self.absolute_alt_msl)

        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            try:
                msg = self.master.recv_match(
                    type="GLOBAL_POSITION_INT",
                    blocking=True,
                    timeout=0.2,
                )
            except Exception:
                break
            if msg is None:
                continue
            self._last_msg_time = time.monotonic()
            alt_msl = msg.alt / 1000.0
            self.relative_alt = msg.relative_alt / 1000.0
            if math.isfinite(alt_msl):
                self.absolute_alt_msl = alt_msl
                return alt_msl

        self.get_logger().warn(
            f"未及时收到GLOBAL_POSITION_INT, EKF原点海拔回退到 {DEFAULT_ORIGIN_ALT_MSL:.1f}m"
        )
        return DEFAULT_ORIGIN_ALT_MSL

    def send_land(self):
        """发送降落指令"""
        if not self.set_ekf_origin_on_start and self._latest_home is None:
            self.get_logger().warn(
                "无GPS原点/HOME_POSITION, 拒绝切换LAND以避免ArduPilot invalid location panic"
            )
            return
        land_id = self._resolve_mode("LAND")
        if land_id is not None:
            self.master.mav.command_long_send(
                self.master.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                land_id,
                0, 0, 0, 0, 0,
            )
            return
        self.get_logger().warn("LAND模式不可用, 未发送MAV_CMD_NAV_LAND以避免无GPS原点panic")

    def send_disarm(self):
        """发送上锁指令"""
        if self.force_disarm:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                21196,
                0, 0, 0, 0, 0,
            )
        else:
            self.master.arducopter_disarm()

    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'master') and self.master:
            try:
                self.master.close()
            except Exception:
                pass
        if rclpy.ok():
            self.get_logger().info("MAVLink桥节点已关闭")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = APMavlinkBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
