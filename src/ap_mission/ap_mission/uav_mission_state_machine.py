#!/usr/bin/env python3
"""UAV 空地协同比赛任务状态机。

仿真任务按比赛顺序执行:
  起飞 → 避让障碍柱(去程) → 穿越方框(去程) → 识别地面码
  → 穿越方框(返程) → 避让障碍柱(返程) → 到UGV最终停车/扎球点
  → 等待并行出发的UGV穿完三个门洞且车和气球停到最终扎球点
  → 搜索气球 → 对准气球 → 扎破气球 → 返回起飞点 → 降落/结束

定位估计通过 ExternalNav 接口反馈给 ArduPilot，状态机始终使用
本地 NED 坐标系（X=北/前，Y=东/右，航点高度为相对起飞点的正向上值）。
"""

import time
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, Float32MultiArray, ColorRGBA
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Vector3, Twist
from visualization_msgs.msg import Marker, MarkerArray

from .mission_config import MISSION_PARAMETER_DEFAULTS, WAYPOINT_REACHED_RADIUS
from .mission_config import WAYPOINT_TIMEOUT  # noqa: F401 (compatibility export)
from .mission_state import State
from .competition_route import COMPETITION_WAYPOINTS_NED
from .flight_control_3d import (
    Controller3DConfig,
    GuidedController3D,
    lookahead_target_3d,
    staged_return_step,
    staged_return_target,
)


class UAVMissionStateMachine(Node):
    """UAV比赛任务状态机 (ExternalNav版)"""

    def __init__(self):
        super().__init__("uav_mission_state_machine")

        # ===== 比赛参数 =====
        for name, default in MISSION_PARAMETER_DEFAULTS:
            self.declare_parameter(name, default)

        self.waypoint_file = str(self.get_parameter("waypoint_file").value)
        self.native_guided_takeoff = bool(self.get_parameter('native_guided_takeoff').value)
        self.native_autopilot_landing = bool(self.get_parameter('native_autopilot_landing').value)

        self.target_altitude = float(self.get_parameter("target_altitude").value)
        self.hover_target_altitude = float(
            self.get_parameter("hover_target_altitude").value
        )
        if self.hover_target_altitude <= 0.0:
            self.hover_target_altitude = self.target_altitude
        self.hover_pwm = int(self.get_parameter("hover_pwm").value)
        self.takeoff_ramp_time = float(self.get_parameter("takeoff_ramp_time").value)
        self.takeoff_min_pwm_offset = int(self.get_parameter("takeoff_min_pwm_offset").value)
        self.takeoff_max_pwm_offset = int(self.get_parameter("takeoff_max_pwm_offset").value)
        self.takeoff_success_altitude = float(self.get_parameter("takeoff_success_altitude").value)
        self.takeoff_settle_pwm = int(self.get_parameter("takeoff_settle_pwm").value)
        self.takeoff_settle_time = float(self.get_parameter("takeoff_settle_time").value)
        self.takeoff_fallback_brake_pwm = int(
            self.get_parameter("takeoff_fallback_brake_pwm").value
        )
        self.takeoff_fallback_ramp_start_pwm = int(
            self.get_parameter("takeoff_fallback_ramp_start_pwm").value
        )
        self.takeoff_fallback_ramp_step_pwm = int(
            self.get_parameter("takeoff_fallback_ramp_step_pwm").value
        )
        self.takeoff_fallback_ramp_step_time = float(
            self.get_parameter("takeoff_fallback_ramp_step_time").value
        )
        self.takeoff_fallback_ramp_max_pwm = int(
            self.get_parameter("takeoff_fallback_ramp_max_pwm").value
        )
        self.takeoff_fallback_liftoff_altitude = float(
            self.get_parameter("takeoff_fallback_liftoff_altitude").value
        )
        self.takeoff_fallback_brake_climb_rate = float(
            self.get_parameter("takeoff_fallback_brake_climb_rate").value
        )
        self.takeoff_fallback_abort_climb_rate = float(
            self.get_parameter("takeoff_fallback_abort_climb_rate").value
        )
        self.takeoff_fallback_emergency_climb_rate = float(
            self.get_parameter("takeoff_fallback_emergency_climb_rate").value
        )
        self.takeoff_fallback_emergency_climb_time = float(
            self.get_parameter("takeoff_fallback_emergency_climb_time").value
        )
        self.takeoff_fallback_max_altitude = float(
            self.get_parameter("takeoff_fallback_max_altitude").value
        )
        self.takeoff_fallback_timeout = float(
            self.get_parameter("takeoff_fallback_timeout").value
        )
        self.takeoff_fallback_mode = str(
            self.get_parameter("takeoff_fallback_mode").value
        ).upper()
        self.takeoff_fallback_handoff_vertical_rate_max = float(
            self.get_parameter("takeoff_fallback_handoff_vertical_rate_max").value
        )
        self.takeoff_fallback_handoff_stable_time = float(
            self.get_parameter("takeoff_fallback_handoff_stable_time").value
        )
        self.takeoff_fallback_ramp_start_pwm = max(
            1400, min(1800, self.takeoff_fallback_ramp_start_pwm)
        )
        self.takeoff_fallback_ramp_step_pwm = max(
            1, min(100, self.takeoff_fallback_ramp_step_pwm)
        )
        self.takeoff_fallback_ramp_step_time = max(
            0.2, min(5.0, self.takeoff_fallback_ramp_step_time)
        )
        self.takeoff_fallback_ramp_max_pwm = max(
            self.takeoff_fallback_ramp_start_pwm,
            min(1800, self.takeoff_fallback_ramp_max_pwm),
        )
        self.takeoff_fallback_liftoff_altitude = max(
            0.05,
            min(self.takeoff_success_altitude, self.takeoff_fallback_liftoff_altitude),
        )
        self.takeoff_fallback_brake_climb_rate = max(
            0.10, self.takeoff_fallback_brake_climb_rate
        )
        self.takeoff_fallback_abort_climb_rate = max(
            self.takeoff_fallback_brake_climb_rate + 0.10,
            self.takeoff_fallback_abort_climb_rate,
        )
        self.takeoff_fallback_emergency_climb_rate = max(
            self.takeoff_fallback_abort_climb_rate + 0.50,
            self.takeoff_fallback_emergency_climb_rate,
        )
        self.takeoff_fallback_emergency_climb_time = max(
            0.20, self.takeoff_fallback_emergency_climb_time
        )
        self.takeoff_fallback_handoff_vertical_rate_max = max(
            0.05, self.takeoff_fallback_handoff_vertical_rate_max
        )
        self.takeoff_fallback_handoff_stable_time = max(
            0.0, self.takeoff_fallback_handoff_stable_time
        )
        if self.takeoff_fallback_mode not in ("LOITER", "ALT_HOLD", "STABILIZE"):
            self.get_logger().warn(
                f"takeoff_fallback_mode={self.takeoff_fallback_mode}无效, 回退ALT_HOLD"
            )
            self.takeoff_fallback_mode = "ALT_HOLD"
        self.takeoff_guided_handoff_time = float(
            self.get_parameter("takeoff_guided_handoff_time").value
        )
        self.takeoff_guided_handoff_climb_rate = float(
            self.get_parameter("takeoff_guided_handoff_climb_rate").value
        )
        self.takeoff_guided_handoff_min_altitude = float(
            self.get_parameter("takeoff_guided_handoff_min_altitude").value
        )
        self.takeoff_guided_handoff_min_altitude = max(
            0.05, min(self.target_altitude, self.takeoff_guided_handoff_min_altitude)
        )
        self.takeoff_guided_handoff_max_rp_deg = float(
            self.get_parameter("takeoff_guided_handoff_max_rp_deg").value
        )
        self.takeoff_guided_handoff_max_local_xy_error = float(
            self.get_parameter("takeoff_guided_handoff_max_local_xy_error").value
        )
        self.takeoff_guided_handoff_max_local_z_error = float(
            self.get_parameter("takeoff_guided_handoff_max_local_z_error").value
        )
        self.enable_local_position_z_consistency = bool(
            self.get_parameter("enable_local_position_z_consistency").value
        )
        self.takeoff_hold_fixed_origin = bool(
            self.get_parameter("takeoff_hold_fixed_origin").value
        )
        self.use_guided_attitude_takeoff_fallback = bool(
            self.get_parameter("use_guided_attitude_takeoff_fallback").value
        )
        self.takeoff_guided_fallback_climb_rate = float(
            self.get_parameter("takeoff_guided_fallback_climb_rate").value
        )
        self.takeoff_guided_fallback_timeout = float(
            self.get_parameter("takeoff_guided_fallback_timeout").value
        )
        self.allow_takeoff_rc_fallback = bool(self.get_parameter("allow_takeoff_rc_fallback").value)
        self.prefer_takeoff_rc_fallback = bool(
            self.get_parameter("prefer_takeoff_rc_fallback").value
        )
        self.max_mission_time = float(self.get_parameter("max_mission_time").value)
        self.max_start_time = float(self.get_parameter("max_start_time").value)
        self.ignore_mission_start = bool(self.get_parameter("ignore_mission_start").value)
        self.waypoint_timeout = max(
            30.0, float(self.get_parameter("waypoint_timeout").value)
        )
        self.allow_waypoint_timeout_skip = bool(
            self.get_parameter("allow_waypoint_timeout_skip").value
        )
        self.ugv_wait_timeout = float(self.get_parameter("ugv_wait_timeout").value)
        self.require_code_success = bool(self.get_parameter("require_code_success").value)
        self.require_ugv_ready_for_strike = bool(
            self.get_parameter("require_ugv_ready_for_strike").value
        )
        self.allow_strike_without_ugv = bool(
            self.get_parameter("allow_strike_without_ugv").value
        )
        self.code_failure_action = str(self.get_parameter("code_failure_action").value).upper()
        if self.code_failure_action not in ("LANDING", "ABORT", "RETURN_HOME"):
            self.get_logger().warn(
                f"code_failure_action={self.code_failure_action}无效, 回退LANDING"
            )
            self.code_failure_action = "LANDING"
        self.code_success_hold_time = max(
            0.0, float(self.get_parameter("code_success_hold_time").value)
        )
        self.code_success_settle_speed = max(
            0.02, float(self.get_parameter("code_success_settle_speed").value)
        )
        self.k_frame = float(self.get_parameter("k_frame").value)
        requested_localization_mode = bool(self.get_parameter("localization_mode").value)
        if not requested_localization_mode:
            self.get_logger().warn(
                "localization_mode=false 已不再支持, 将按 ExternalNav + GUIDED 主线执行"
            )
        self.localization_mode = True
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
        self.k_pos_p = float(self.get_parameter("k_pos_p").value)
        self.k_yaw_p = float(self.get_parameter("k_yaw_p").value)
        self.max_pitch_pwm = int(self.get_parameter("max_pitch_pwm").value)
        self.max_roll_pwm = int(self.get_parameter("max_roll_pwm").value)
        self.guided_mode_timeout = float(self.get_parameter("guided_mode_timeout").value)
        self.guided_target_period = float(self.get_parameter("guided_target_period").value)
        self.guided_nav_control_mode = str(
            self.get_parameter("guided_nav_control_mode").value
        ).lower()
        if self.guided_nav_control_mode not in ("position", "velocity"):
            self.get_logger().warn(
                f"guided_nav_control_mode={self.guided_nav_control_mode}无效, 回退velocity"
            )
            self.guided_nav_control_mode = "velocity"
        self.guided_3d_enabled = bool(self.get_parameter("guided_3d_enabled").value)
        self.guided_controller_3d = GuidedController3D(
            Controller3DConfig(
                kp=float(self.get_parameter("guided_nav_k_xy").value),
                kd=float(self.get_parameter("guided_nav_cross_track_d").value),
                max_speed=float(self.get_parameter("guided_nav_speed_max").value),
                max_vertical_speed=float(self.get_parameter("guided_nav_z_speed_max").value),
            )
        )
        self.guided_position_strategy = str(
            self.get_parameter("guided_position_strategy").value
        ).lower()
        if self.guided_position_strategy not in ("direct", "segment"):
            self.get_logger().warn(
                f"guided_position_strategy={self.guided_position_strategy}无效, 回退segment"
            )
            self.guided_position_strategy = "segment"
        self.waypoint_marker_frame = str(self.get_parameter("waypoint_marker_frame").value)
        self.guided_nav_speed_max = max(
            0.02, float(self.get_parameter("guided_nav_speed_max").value)
        )
        self.guided_nav_z_speed_max = max(
            0.03, float(self.get_parameter("guided_nav_z_speed_max").value)
        )
        self.guided_nav_k_xy = max(0.05, float(self.get_parameter("guided_nav_k_xy").value))
        self.guided_nav_k_z = max(0.05, float(self.get_parameter("guided_nav_k_z").value))
        self.guided_nav_min_along_speed = max(
            0.0,
            min(
                self.guided_nav_speed_max,
                float(self.get_parameter("guided_nav_min_along_speed").value),
            ),
        )
        self.guided_nav_cross_track_k = max(
            0.05, float(self.get_parameter("guided_nav_cross_track_k").value)
        )
        self.guided_nav_cross_track_d = max(
            0.0, float(self.get_parameter("guided_nav_cross_track_d").value)
        )
        self.guided_nav_cross_track_speed_max = max(
            0.02, float(self.get_parameter("guided_nav_cross_track_speed_max").value)
        )
        self.guided_nav_cross_track_stop = max(
            0.15, float(self.get_parameter("guided_nav_cross_track_stop").value)
        )
        self.guided_nav_attitude_brake_rp_deg = max(
            8.0, float(self.get_parameter("guided_nav_attitude_brake_rp_deg").value)
        )
        self.guided_position_brake_speed_xy = max(
            self.guided_nav_speed_max,
            float(self.get_parameter("guided_position_brake_speed_xy").value),
        )
        self.guided_position_brake_resume_speed_xy = max(
            0.02,
            min(
                self.guided_position_brake_speed_xy,
                float(self.get_parameter("guided_position_brake_resume_speed_xy").value),
            ),
        )
        self.guided_velocity_brake_speed_xy = max(
            self.guided_nav_speed_max,
            float(self.get_parameter("guided_velocity_brake_speed_xy").value),
        )
        self.guided_velocity_resume_speed_xy = max(
            0.02,
            min(
                self.guided_velocity_brake_speed_xy,
                float(self.get_parameter("guided_velocity_resume_speed_xy").value),
            ),
        )
        self.guided_velocity_brake_damping = max(
            0.0, float(self.get_parameter("guided_velocity_brake_damping").value)
        )
        self.guided_velocity_brake_cmd_max = max(
            0.0, float(self.get_parameter("guided_velocity_brake_cmd_max").value)
        )
        self.external_nav_velocity_lpf_alpha = max(
            0.05,
            min(
                1.0,
                float(
                    self.get_parameter(
                        "external_nav_velocity_lpf_alpha"
                    ).value
                ),
            ),
        )
        self.waypoint_reached_speed_xy_max = max(
            0.03, float(self.get_parameter("waypoint_reached_speed_xy_max").value)
        )
        self.waypoint_arrival_settle_time = max(
            0.0, float(self.get_parameter("waypoint_arrival_settle_time").value)
        )
        self.guided_hold_speed_max = max(
            0.01,
            min(
                self.guided_nav_speed_max,
                float(self.get_parameter("guided_hold_speed_max").value),
            ),
        )
        self.guided_velocity_slew_xy = max(
            0.02, float(self.get_parameter("guided_velocity_slew_xy").value)
        )
        self.guided_velocity_slew_z = max(
            0.02, float(self.get_parameter("guided_velocity_slew_z").value)
        )
        self.guided_alt_recovery_z_speed_max = max(
            self.guided_nav_z_speed_max,
            float(self.get_parameter("guided_alt_recovery_z_speed_max").value),
        )
        self.hover_stable_time = float(self.get_parameter("hover_stable_time").value)
        self.hover_stable_timeout = float(self.get_parameter("hover_stable_timeout").value)
        self.hover_climb_rate = float(self.get_parameter("hover_climb_rate").value)
        self.hover_alt_tolerance = float(self.get_parameter("hover_alt_tolerance").value)
        self.hover_xy_tolerance = float(self.get_parameter("hover_xy_tolerance").value)
        self.hover_speed_xy_max = float(self.get_parameter("hover_speed_xy_max").value)
        self.hover_vertical_rate_max = float(self.get_parameter("hover_vertical_rate_max").value)
        self.hover_max_rp_deg = float(self.get_parameter("hover_max_rp_deg").value)
        self.hover_guided_velocity_hold = bool(
            self.get_parameter("hover_guided_velocity_hold").value
        )
        self.hover_hold_speed_xy_max = max(
            0.01, float(self.get_parameter("hover_hold_speed_xy_max").value)
        )
        self.hover_hold_z_speed_max = max(
            0.01, float(self.get_parameter("hover_hold_z_speed_max").value)
        )
        self.hover_hold_descent_speed_max = max(
            self.hover_hold_z_speed_max,
            float(self.get_parameter("hover_hold_descent_speed_max").value),
        )
        self.skip_hover_stable_after_takeoff = bool(
            self.get_parameter("skip_hover_stable_after_takeoff").value
        )
        self.takeoff_localization_grace_time = max(
            0.0, float(self.get_parameter("takeoff_localization_grace_time").value)
        )
        self.airborne_min_altitude = float(self.get_parameter("airborne_min_altitude").value)
        self.airborne_min_altitude_grace = float(
            self.get_parameter("airborne_min_altitude_grace").value
        )
        self.airborne_max_rp_deg = float(self.get_parameter("airborne_max_rp_deg").value)
        self.airborne_attitude_grace = float(
            self.get_parameter("airborne_attitude_grace").value
        )
        self.hover_local_position_max_xy_error = float(
            self.get_parameter("hover_local_position_max_xy_error").value
        )
        self.guided_step_distance = float(self.get_parameter("guided_step_distance").value)
        self.guided_path_lookahead = max(
            0.05, float(self.get_parameter("guided_path_lookahead").value)
        )
        self.guided_path_recovery_lookahead = max(
            0.03, float(self.get_parameter("guided_path_recovery_lookahead").value)
        )
        self.guided_path_cross_track_warn = max(
            0.10, float(self.get_parameter("guided_path_cross_track_warn").value)
        )
        self.enable_local_position_consistency = bool(
            self.get_parameter("enable_local_position_consistency").value
        )
        self.local_position_max_xy_error = float(
            self.get_parameter("local_position_max_xy_error").value
        )
        self.local_position_stale_timeout = float(
            self.get_parameter("local_position_stale_timeout").value
        )
        self.external_nav_stale_timeout = max(
            0.5, float(self.get_parameter("external_nav_stale_timeout").value)
        )
        self.localization_fault_hold_time = float(
            self.get_parameter("localization_fault_hold_time").value
        )
        self.localization_failure_action = str(
            self.get_parameter("localization_failure_action").value
        ).upper()
        self.landing_strategy = str(self.get_parameter("landing_strategy").value).upper()
        self.landing_descent_pwm = int(self.get_parameter("landing_descent_pwm").value)
        self.landing_disarm_delay = float(self.get_parameter("landing_disarm_delay").value)
        self.landing_guided_descent_rate = max(
            0.0, float(self.get_parameter("landing_guided_descent_rate").value)
        )
        self.landing_xy_speed_max = max(
            0.02, float(self.get_parameter("landing_xy_speed_max").value)
        )
        self.landing_xy_damping = max(
            0.0, float(self.get_parameter("landing_xy_damping").value)
        )
        self.landing_ground_altitude = max(
            0.05, float(self.get_parameter("landing_ground_altitude").value)
        )
        self.simulate_balloon_puncture_on_detection = bool(
            self.get_parameter("simulate_balloon_puncture_on_detection").value
        )
        self.post_puncture_return_home = bool(
            self.get_parameter("post_puncture_return_home").value
        )
        self.post_puncture_return_altitude = max(
            0.3, float(self.get_parameter("post_puncture_return_altitude").value)
        )
        self.post_puncture_return_timeout = max(
            5.0, float(self.get_parameter("post_puncture_return_timeout").value)
        )
        self.post_puncture_return_step_distance = max(
            0.05, float(self.get_parameter("post_puncture_return_step_distance").value)
        )
        self.post_puncture_return_speed_min = max(
            0.0,
            min(
                self.guided_nav_speed_max,
                float(self.get_parameter("post_puncture_return_speed_min").value),
            ),
        )
        self.post_puncture_home_brake_radius = max(
            0.2, float(self.get_parameter("post_puncture_home_brake_radius").value)
        )
        self.post_puncture_home_brake_damping = max(
            0.0,
            float(self.get_parameter("post_puncture_home_brake_damping").value),
        )
        self.post_puncture_home_radius = max(
            0.05, float(self.get_parameter("post_puncture_home_radius").value)
        )
        self.post_puncture_home_speed_max = max(
            0.01, float(self.get_parameter("post_puncture_home_speed_max").value)
        )
        self.post_puncture_home_settle_time = max(
            0.0, float(self.get_parameter("post_puncture_home_settle_time").value)
        )
        self.post_puncture_climb_tolerance = max(
            0.05, float(self.get_parameter("post_puncture_climb_tolerance").value)
        )
        self.post_puncture_climb_settle_time = max(
            0.0, float(self.get_parameter("post_puncture_climb_settle_time").value)
        )
        self.safe_stop_mode = str(self.get_parameter("safe_stop_mode").value).upper()
        self.safe_stop_descent_pwm = int(self.get_parameter("safe_stop_descent_pwm").value)
        self.safe_stop_disarm_delay = float(self.get_parameter("safe_stop_disarm_delay").value)
        self.complete_requires_disarmed = bool(
            self.get_parameter("complete_requires_disarmed").value
        )
        self.enable_sim_truth_monitor = bool(self.get_parameter("enable_sim_truth_monitor").value)
        self.sim_truth_max_external_nav_xy_error = float(
            self.get_parameter("sim_truth_max_external_nav_xy_error").value
        )
        self.sim_truth_stale_timeout = float(self.get_parameter("sim_truth_stale_timeout").value)
        self.sim_geofence_x_min = float(self.get_parameter("sim_geofence_x_min").value)
        self.sim_geofence_x_max = float(self.get_parameter("sim_geofence_x_max").value)
        self.sim_geofence_y_min = float(self.get_parameter("sim_geofence_y_min").value)
        self.sim_geofence_y_max = float(self.get_parameter("sim_geofence_y_max").value)
        self.sim_geofence_alt_max = float(self.get_parameter("sim_geofence_alt_max").value)

        # ===== 状态机 =====
        self.state = State.IDLE
        self.prev_state = None
        self.mission_start_time = None
        self.state_start_time = time.time()

        # ===== 飞控状态 =====
        self.is_connected = False
        self.is_armed = False
        self.current_mode = "UNKNOWN"
        self.current_alt = 0.0
        self.battery_voltage = 0.0
        self.ekf_ok = False

        # ===== 起飞辅助变量 =====
        self.takeoff_start_alt = None
        self._takeoff_xy = None
        self._takeoff_command_sent = False
        self._takeoff_use_guided_attitude_fallback = False
        self._takeoff_use_rc_fallback = False
        self._takeoff_fallback_start_time = 0.0
        self._takeoff_settle_until = 0.0
        self._takeoff_fallback_last_alt = None
        self._takeoff_fallback_last_time = 0.0
        self._takeoff_fallback_climb_rate = 0.0
        self._takeoff_fallback_braking = False
        self._takeoff_fallback_ramp_pwm = 0
        self._takeoff_fallback_next_ramp_time = 0.0
        self._takeoff_fallback_liftoff_detected = False
        self._takeoff_fallback_handoff_stable_since = 0.0
        self._takeoff_fallback_high_rate_since = 0.0
        self._guided_handoff_done = False
        self._last_rc_log_time = 0.0
        self._last_alt_log_time = 0.0
        self._ekf_abort_triggered = False      # B1修复: 防ABORT死循环
        self._last_connected_time = time.time()  # B3修复: 飞控断连检测

        # ===== ARMING 限流变量 (C4修复) =====
        self._last_mode_req_time = 0.0       # 上次发送模式切换请求的时间

        # ===== 任务控制 =====
        self.mission_started = False
        self._mission_success = False
        self._mission_end_reason = ""

        # ===== 视觉状态 =====
        self.red_ball_found = False
        self.red_ball_aligned = False
        self.red_ball_cx = 0.0
        self.red_ball_cy = 0.0
        self.code_detected = False
        self.code_data = ""
        self.frame_found = False
        self.frame_cx = 0.0
        self.frame_cy = 0.0
        self.frame_diag = 0.0

        # ===== ExternalNav 状态 =====
        self.external_nav_ready = False
        self.fcu_home_ready = False
        self.fcu_landed = False
        self._last_landed_time = 0.
        self._landing_touchdown_since = None
        self._landing_settled_since = None
        self.external_nav_position = None  # (x, y, z) NED
        self.local_position = None  # FCU LOCAL_POSITION_NED (x, y, z)
        self.local_velocity = (0.0, 0.0, 0.0)  # FCU LOCAL_POSITION_NED velocity
        self.sim_truth_position = None  # Gazebo truth local NED (x, y, z)
        self.attitude_rpy = None  # FCU roll/pitch/yaw (rad)
        self._last_external_nav_pose_time = 0.0
        self._last_local_pos_time = 0.0
        self._last_local_velocity_time = 0.0
        self._last_sim_truth_time = 0.0
        self._last_attitude_time = 0.0
        self._localization_fault_since = None
        self._last_external_nav_motion_sample = None
        self._external_nav_velocity = (0.0, 0.0, 0.0)
        self._external_nav_speed_xy = 0.0
        self._external_nav_vertical_rate = 0.0
        self._hover_stable_since = None
        self._last_hover_alt_update = 0.0
        self._last_hover_wait_log = 0.0
        self._low_airborne_since = None
        self._bad_attitude_since = None

        # ===== 航点导航 =====
        self._wpt_index = 0                          # 当前目标航点索引
        self._wpt_waypoints = self._build_waypoints()    # 航点列表
        self._wpt_deadline = 0.0                     # 当前航点超时
        self._guided_mode_deadline = 0.0
        self._last_guided_mode_req_time = 0.0
        self._last_guided_target_time = 0.0
        self._guided_hold_target = None              # (x_north, y_east, altitude_m, label)
        self._rc_released_for_guided = False
        self._guided_handoff_level_since = 0.0
        self._guided_handoff_complete_time = 0.0
        self._last_guided_handoff_log = 0.0
        self._takeoff_localization_grace_until = 0.0
        self._guided_segment_progress = {}
        self._guided_segment_progress_time = {}
        self._guided_position_brake_target = None
        self._guided_velocity_brake_active = False
        self._last_waypoint_timeout_log = 0.0
        self._last_position_brake_log = 0.0
        self._last_velocity_brake_log = 0.0
        self._last_waypoint_arrival_brake_log = 0.0
        self._waypoint_arrival_stable_since = None
        self._last_guided_velocity_cmd = None
        self._last_guided_velocity_cmd_time = 0.0

        # ===== UGV 协同 =====
        self.ugv_in_strike_zone = False
        self.strike_permission = False
        self.code_task_done = False
        self.return_frame_done = False
        self.return_pillars_done = False
        self.uav_back_in_strike_zone = False
        self._ugv_timeout_logged = False
        self._code_success_since = None
        self._last_code_settle_log = 0.0
        self.balloon_punctured = False
        self._post_strike_deadline = 0.0
        self._post_strike_phase = "IDLE"
        self._post_strike_anchor_xy = None
        self._post_strike_climb_stable_since = None
        self._post_strike_home_stable_since = None

        # ===== 发布者 =====
        self.mode_pub = self.create_publisher(String, "/ap/cmd/set_mode", 10)
        self.arm_pub = self.create_publisher(Bool, "/ap/cmd/arm", 10)
        self.land_pub = self.create_publisher(Bool, "/ap/cmd/land", 10)
        self.rc_pub = self.create_publisher(Joy, "/ap/cmd/rc_override", 10)
        self.rc_release_pub = self.create_publisher(Bool, "/ap/cmd/release_rc_override", 10)
        self.guided_pos_pub = self.create_publisher(Point, "/ap/cmd/guided_position", 10)
        self.guided_vel_pub = self.create_publisher(Twist, "/ap/cmd/guided_velocity", 10)
        self.guided_level_climb_pub = self.create_publisher(
            Float32, "/ap/cmd/guided_level_climb_rate", 10
        )
        self.takeoff_pub = self.create_publisher(Float32, "/ap/cmd/takeoff_alt", 10)
        self.state_pub = self.create_publisher(String, "/mission/uav_state", 10)
        self.strike_perm_pub = self.create_publisher(Bool, "/mission/strike_permission", 10)
        self.balloon_puncture_pub = self.create_publisher(Bool, "/mission/balloon_punctured", 10)
        self.ugv_start_pub = self.create_publisher(Bool, "/mission/ugv_start", 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, "/mission/waypoints", 10)

        # ===== 订阅者 =====
        self.create_subscription(Bool, "/ap/connected", self.connected_cb, 10)
        self.create_subscription(String, "/ap/mode", self.mode_cb, 10)
        self.create_subscription(Bool, "/ap/armed", self.armed_cb, 10)
        self.create_subscription(Float32, "/ap/relative_alt", self.alt_cb, 10)
        self.create_subscription(Float32, "/ap/battery_voltage", self.batt_cb, 10)
        self.create_subscription(Vector3, "/ap/roll_pitch_yaw", self.attitude_cb, 10)
        self.create_subscription(Bool, "/ap/ekf_ok", self.ekf_cb, 10)
        self.create_subscription(Float32MultiArray, "/vision/red_ball", self.vision_cb, 10)
        self.create_subscription(String, "/vision/code_result", self.code_cb, 10)
        self.create_subscription(Bool, "/vision/code_found", self.code_found_cb, 10)
        self.create_subscription(Float32MultiArray, "/vision/frame_box", self.frame_cb, 10)
        self.create_subscription(Bool, "/mission/start", self.start_cb, 10)
        self.create_subscription(Bool, "/mission/ugv_in_strike_zone", self.ugv_cb, 10)
        self.create_subscription(Bool, "/mission/abort", self.abort_cb, 10)
        self.create_subscription(Bool, '/mission/land', self.land_request_cb, 10)
        self.create_subscription(Bool, "/ap/external_nav/ready", self.external_nav_ready_cb, 10)
        self.create_subscription(Bool, '/ap/home_ready', self.home_ready_cb, 10)
        self.create_subscription(Bool, '/ap/landed', self.landed_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/ap/external_nav/vision_pose",
            self.external_nav_pose_cb,
            10,
        )
        self.create_subscription(Point, "/ap/local_pos", self.local_pos_cb, 10)
        self.create_subscription(Twist, "/ap/local_velocity", self.local_velocity_cb, 10)
        if self.enable_sim_truth_monitor:
            self.create_subscription(PoseStamped, "/sim/drone_pose_ned", self.sim_truth_cb, 10)
        # ===== 定时器 (10Hz) =====
        self.timer = self.create_timer(0.1, self.state_machine_loop)

        # 发布初始状态
        self.state_pub.publish(String(data=self.state))

        self.get_logger().info(
            f"UAV任务状态机(ExternalNav版)已启动: "
            f"航点高度={self.target_altitude}m, "
            f"悬停验收高度={self.hover_target_altitude}m, "
            f"hover_pwm={self.hover_pwm}"
        )

    def _build_waypoints(self):
        """Build mission waypoints from target_altitude.

        The competition route uses 1.0m nominal altitude. For low-altitude
        target_altitude scales the transit waypoints without changing the
        route shape.
        """
        if getattr(self, 'waypoint_file', ''):
            from ap_navigation.route_io import load_route
            route = list(load_route(self.waypoint_file))
            self.target_altitude = route[0][2]
            if self.native_guided_takeoff:
                self.hover_target_altitude = self.target_altitude
            return route
        transit_alt = max(0.3, min(self.target_altitude, 1.3))
        strike_alt = min(0.8, transit_alt)
        waypoints = []
        for x, y, z, label in COMPETITION_WAYPOINTS_NED:
            if label == "LAND":
                z = 0.0
            elif label == "STRIKE_ZONE":
                z = strike_alt
            elif label == "BALLOON_ALT":
                z = min(z, strike_alt)
            else:
                z = transit_alt
            waypoints.append((x, y, z, label))
        return waypoints

    def _find_waypoint_index(self, label):
        for idx, (_, _, _, item_label) in enumerate(self._wpt_waypoints):
            if item_label == label:
                return idx
        return None

    def _resume_waypoints_through_label(self, label):
        """Resume queued intermediate points before the next required task."""
        idx = self._find_waypoint_index(label)
        if idx is None:
            self.get_logger().error(f"任务航点缺少 {label}, 无法继续比赛流程")
            self.transition(State.ABORT)
            return False
        if not 0 <= self._wpt_index <= idx:
            self.get_logger().error(f"当前航点已越过任务目标 {label}, 无法继续比赛流程")
            self.transition(State.ABORT)
            return False
        self._wpt_deadline = time.time() + self.waypoint_timeout
        self._guided_hold_target = self._wpt_waypoints[self._wpt_index]
        self._waypoint_arrival_stable_since = None
        return True

    # ===== 回调 =====
    def connected_cb(self, msg):
        self.is_connected = msg.data
        self._last_connected_time = time.time()

    def mode_cb(self, msg): self.current_mode = msg.data

    def armed_cb(self, msg): self.is_armed = msg.data

    def alt_cb(self, msg): self.current_alt = msg.data

    def batt_cb(self, msg): self.battery_voltage = msg.data

    def ekf_cb(self, msg): self.ekf_ok = msg.data

    def external_nav_ready_cb(self, msg): self.external_nav_ready = msg.data

    def attitude_cb(self, msg):
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            return
        self.attitude_rpy = (float(msg.x), float(msg.y), float(msg.z))
        self._last_attitude_time = time.time()

    def start_cb(self, msg):
        if msg.data and not self.mission_started:
            if self.ignore_mission_start:
                self.get_logger().info("收到比赛开始信号: 已忽略")
                return
            self.get_logger().info("收到比赛开始信号!")
            self._reset_rule_gates()
            self._mission_success = False
            self._mission_end_reason = ""
            self.mission_started = True

    def ugv_cb(self, msg): self.ugv_in_strike_zone = msg.data

    def vision_cb(self, msg):
        if len(msg.data) >= 4:
            self.red_ball_found = msg.data[0] > 0.5
            # 修复: 无数据时默认值改为0.0 (居中), 避免虚拟边缘偏移导致失控振荡
            self.red_ball_cx = msg.data[1] if len(msg.data) > 1 else 0.0
            self.red_ball_cy = msg.data[2] if len(msg.data) > 2 else 0.0
            self.red_ball_aligned = abs(self.red_ball_cx) < 0.08 and abs(self.red_ball_cy) < 0.08

    def code_cb(self, msg): self.code_data = msg.data
    def code_found_cb(self, msg): self.code_detected = msg.data

    def _reset_rule_gates(self):
        self.code_task_done = False
        self.return_frame_done = False
        self.return_pillars_done = False
        self.uav_back_in_strike_zone = False
        self.strike_permission = False
        self._ugv_timeout_logged = False
        self._ugv_start_sent = False
        self.balloon_punctured = False
        self._code_success_since = None
        self._last_code_settle_log = 0.0
        self._post_strike_phase = "IDLE"
        self._post_strike_anchor_xy = None
        self._post_strike_climb_stable_since = None
        self._post_strike_home_stable_since = None
        self._publish_strike_permission(False)
        self._publish_balloon_punctured(False)
        self.ugv_start_pub.publish(Bool(data=False))
        self._guided_segment_progress.clear()
        self._guided_segment_progress_time.clear()
        self._guided_position_brake_target = None

    def frame_cb(self, msg):
        if len(msg.data) >= 4:
            self.frame_found = msg.data[0] > 0.5
            self.frame_cx = msg.data[1]
            self.frame_cy = msg.data[2]
            self.frame_diag = msg.data[3]

    def external_nav_pose_cb(self, msg):
        p = msg.pose.pose
        # NaN/Inf保护: ExternalNav退化时可能产生无效值, 传播到P控制器会导致巨大RC偏移
        if not all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z)):
            self.get_logger().warn("ExternalNav位置NaN/Inf! 忽略", throttle_duration_sec=1.0)
            return
        self.external_nav_position = (
            p.position.x,
            p.position.y,
            p.position.z,
        )
        now = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        sample_time = stamp if stamp > 0. else now
        if self._last_external_nav_motion_sample is not None:
            prev_t, prev_x, prev_y, prev_z = self._last_external_nav_motion_sample
            dt = sample_time - prev_t
            if dt >= 0.02:
                vx = (self.external_nav_position[0] - prev_x) / dt
                vy = (self.external_nav_position[1] - prev_y) / dt
                vz = (self.external_nav_position[2] - prev_z) / dt
                if all(math.isfinite(v) for v in (vx, vy, vz)):
                    alpha = self.external_nav_velocity_lpf_alpha
                    prev_vx, prev_vy, prev_vz = self._external_nav_velocity
                    self._external_nav_velocity = (
                        alpha * vx + (1.0 - alpha) * prev_vx,
                        alpha * vy + (1.0 - alpha) * prev_vy,
                        alpha * vz + (1.0 - alpha) * prev_vz,
                    )
                    speed_xy = math.hypot(vx, vy)
                    self._external_nav_speed_xy = (
                        alpha * speed_xy + (1.0 - alpha) * self._external_nav_speed_xy
                    )
                    self._external_nav_vertical_rate = (
                        alpha * (-vz) + (1.0 - alpha) * self._external_nav_vertical_rate
                    )
        self._last_external_nav_motion_sample = (
            sample_time,
            self.external_nav_position[0],
            self.external_nav_position[1],
            self.external_nav_position[2],
        )
        self._last_external_nav_pose_time = now

    def home_ready_cb(self, msg):
        self.fcu_home_ready = bool(msg.data)

    def land_request_cb(self, msg):
        if not msg.data or not self.is_armed:
            return
        self._mission_success = False
        self._mission_end_reason = '用户请求受控降落，任务未完成'
        self.mission_start_time = None
        self.mission_started = False
        if self.state != State.LANDING:
            self.transition(State.LANDING)

    def landed_cb(self, msg):
        self.fcu_landed = bool(msg.data)
        self._last_landed_time = time.time()

    def local_pos_cb(self, msg: Point):
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self.get_logger().warn("FCU local位置NaN/Inf! 忽略", throttle_duration_sec=1.0)
            return
        self.local_position = (float(msg.x), float(msg.y), float(msg.z))
        self._last_local_pos_time = time.time()

    def local_velocity_cb(self, msg: Twist):
        values = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn("FCU local速度NaN/Inf! 忽略", throttle_duration_sec=1.0)
            return
        self.local_velocity = values
        self._last_local_velocity_time = time.time()

    def sim_truth_cb(self, msg: PoseStamped):
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.get_logger().warn("Gazebo真值位置NaN/Inf! 忽略", throttle_duration_sec=1.0)
            return
        self.sim_truth_position = (float(p.x), float(p.y), float(p.z))
        self._last_sim_truth_time = time.time()

    def abort_cb(self, msg):
        if msg.data:
            self.get_logger().warn("收到外部中止信号!")
            self.transition(State.ABORT)

    # ===== 状态机 =====
    def transition(self, new_state):
        if new_state == self.state:
            return
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self._low_airborne_since = None
        self._bad_attitude_since = None
        elapsed = time.time() - (self.mission_start_time or time.time())
        self.get_logger().info(f"[状态机] {self.prev_state} → {self.state} (耗时={elapsed:.1f}s)")
        self.state_pub.publish(String(data=self.state))
        if new_state in (State.ARMING, State.HOVER_STABLE):
            self._rc_released_for_guided = False
            if new_state == State.ARMING or not self.localization_mode:
                self._guided_hold_target = None
            if new_state == State.HOVER_STABLE:
                self._hover_stable_since = None
                self._last_hover_alt_update = time.time()
                self._last_hover_wait_log = 0.0
        if new_state == State.TAKEOFF_RAMP:
            self.takeoff_start_alt = None
            self._takeoff_xy = None
            self._takeoff_command_sent = False
            self._takeoff_use_guided_attitude_fallback = False
            self._takeoff_use_rc_fallback = False
            self._takeoff_fallback_start_time = 0.0
            self._takeoff_settle_until = 0.0
            self._takeoff_fallback_last_alt = None
            self._takeoff_fallback_last_time = 0.0
            self._takeoff_fallback_climb_rate = 0.0
            self._takeoff_fallback_braking = False
            self._takeoff_fallback_ramp_pwm = 0
            self._takeoff_fallback_next_ramp_time = 0.0
            self._takeoff_fallback_liftoff_detected = False
            self._takeoff_fallback_handoff_stable_since = 0.0
            self._takeoff_fallback_high_rate_since = 0.0
            self._guided_handoff_done = False
            self._guided_handoff_level_since = 0.0
            self._guided_handoff_complete_time = 0.0
            self._last_guided_handoff_log = 0.0
        if new_state == State.LANDING and not self._mission_success:
            self._mission_end_reason = self._mission_end_reason or f"未完成任务提前降落(prev={self.prev_state})"
        if new_state == State.LANDING:
            self._landing_touchdown_since = None
            self._landing_settled_since = None
            self._last_guided_target_time = 0.0
            self._last_guided_velocity_cmd = None
            self._last_guided_velocity_cmd_time = 0.0
        if new_state == State.ABORT:
            self._mission_success = False
            self._mission_end_reason = self._mission_end_reason or f"任务中止(prev={self.prev_state})"
        if new_state == State.WAYPOINT_NAV:
            self._guided_mode_deadline = time.time() + self.guided_mode_timeout
            self._last_guided_target_time = 0.0
            self._guided_target_log_time = 0.0
            self._last_waypoint_timeout_log = 0.0
            self._waypoint_arrival_stable_since = None
        if new_state == State.CODE_SEARCH:
            self._code_success_since = None
            self._last_code_settle_log = 0.0
        if new_state == State.RETURN_HOME_AFTER_STRIKE:
            self._guided_mode_deadline = time.time() + self.guided_mode_timeout
            self._post_strike_deadline = time.time() + self.post_puncture_return_timeout
            self._last_guided_target_time = 0.0
            self._guided_target_log_time = 0.0
            self._post_strike_phase = "CLIMB"
            self._post_strike_anchor_xy = None
            self._post_strike_climb_stable_since = None
            self._post_strike_home_stable_since = None

    def _trigger_localization_fault(self, reason):
        """定位闭环异常保护。

        只在异常持续超过 localization_fault_hold_time 后动作, 避免单帧抖动误触发。
        """
        now = time.time()
        if self._localization_fault_since is None:
            self._localization_fault_since = now
            self.get_logger().warn(f"定位一致性异常待确认: {reason}")
            return False
        if now - self._localization_fault_since < self.localization_fault_hold_time:
            return False

        self.get_logger().error(f"定位一致性异常持续存在: {reason}")
        nav_position = self._navigation_position()
        if nav_position is not None:
            cur_x, cur_y, _ = nav_position
            self._guided_hold_target = (cur_x, cur_y, max(self._nav_altitude(), 0.3), "LOCALIZATION_FAULT")
        if self.localization_failure_action in ("SAFE_STOP", "ALT_HOLD", "HOLD"):
            self.mode_pub.publish(String(data=self.safe_stop_mode))
            self._publish_rc(1500, 1500, self._safe_stop_throttle(), 1500)
            self.transition(State.SAFE_STOP)
        elif self.localization_failure_action == "ABORT":
            self.mode_pub.publish(String(data="ALT_HOLD"))
            self.transition(State.ABORT)
        else:
            self.transition(State.LANDING)
        return True

    def _check_localization_consistency(self):
        """检查 ExternalNav、FCU local NED 和仿真真值是否已经明显失配。"""
        now = time.time()
        reasons = []

        if (
            self.enable_local_position_consistency
            and self.external_nav_position is not None
            and self.local_position is not None
            and now - self._last_local_pos_time <= self.local_position_stale_timeout
        ):
            sx, sy, _ = self.external_nav_position
            lx, ly, _ = self.local_position
            err = math.hypot(lx - sx, ly - sy)
            if err > self.local_position_max_xy_error:
                reasons.append(
                    f"FCU local与ExternalNav相差{err:.2f}m "
                    f"local=({lx:.2f},{ly:.2f}) external_nav=({sx:.2f},{sy:.2f})"
                )

        if (
            self.enable_sim_truth_monitor
            and self.sim_truth_position is not None
            and now - self._last_sim_truth_time <= self.sim_truth_stale_timeout
        ):
            tx, ty, tz = self.sim_truth_position
            truth_alt = -tz if tz < 0.0 else tz
            if (
                tx < self.sim_geofence_x_min or tx > self.sim_geofence_x_max
                or ty < self.sim_geofence_y_min or ty > self.sim_geofence_y_max
                or truth_alt > self.sim_geofence_alt_max
            ):
                reasons.append(
                    f"Gazebo真值越界 truth=({tx:.2f},{ty:.2f},alt={truth_alt:.2f})"
                )
            if self.external_nav_position is not None:
                sx, sy, _ = self.external_nav_position
                err = math.hypot(tx - sx, ty - sy)
                if err > self.sim_truth_max_external_nav_xy_error:
                    reasons.append(
                        f"Gazebo真值与ExternalNav相差{err:.2f}m "
                        f"truth=({tx:.2f},{ty:.2f}) external_nav=({sx:.2f},{sy:.2f})"
                    )

        if reasons:
            return self._trigger_localization_fault("; ".join(reasons))
        self._localization_fault_since = None
        return False

    def state_machine_loop(self):
        """10Hz 主循环"""
        self.state_pub.publish(String(data=self.state))

        # H3修复: 飞行模式异常检测 — 外部手动接管时自动ABORT
        _mission_states = (
            State.WAIT_EXTERNAL_NAV, State.TAKEOFF_RAMP, State.HOVER_STABLE,
            State.WAYPOINT_NAV, State.CODE_SEARCH, State.WAIT_UGV_READY,
            State.BALLOON_SEARCH, State.BALLOON_ALIGN, State.PUNCTURE_DESCEND,
            State.PUNCTURE_CONFIRM, State.RETURN_HOME_AFTER_STRIKE,
            State.LANDING, State.SAFE_STOP,
        )
        if self.is_armed and self.state in _mission_states:
            if self.current_mode not in ("GUIDED", "LOITER", "ALT_HOLD", "STABILIZE", "LAND"):
                self.get_logger().error(
                    f"飞行模式异常切换: {self.current_mode}, 疑似手动接管!"
                )
                self.transition(State.ABORT)
                return

        _airborne_states = (
            State.TAKEOFF_RAMP, State.HOVER_STABLE, State.WAYPOINT_NAV,
            State.CODE_SEARCH, State.WAIT_UGV_READY, State.BALLOON_SEARCH,
            State.BALLOON_ALIGN, State.PUNCTURE_DESCEND, State.PUNCTURE_CONFIRM,
            State.RETURN_HOME_AFTER_STRIKE, State.LANDING, State.SAFE_STOP,
        )
        if (
            not self.is_armed
            and self.state in _airborne_states
            and self.state not in (State.LANDING, State.SAFE_STOP, State.COMPLETE)
            and self.mission_started
            and self._elapsed() > 1.0
        ):
            self.get_logger().error(f"飞控意外上锁: state={self.state}, mode={self.current_mode}")
            self.transition(State.ABORT)
            return

        protected_airborne_states = (
            State.HOVER_STABLE, State.WAYPOINT_NAV, State.CODE_SEARCH,
            State.WAIT_UGV_READY, State.BALLOON_SEARCH, State.BALLOON_ALIGN,
            State.PUNCTURE_DESCEND,
        )
        if self.is_armed and self.state in protected_airborne_states:
            now = time.time()
            min_airborne_alt = self.airborne_min_altitude
            min_airborne_grace = self.airborne_min_altitude_grace
            if self.state == State.HOVER_STABLE and not self._guided_handoff_done:
                # During RC/STABILIZE liftoff to GUIDED handoff the vehicle can
                # dip briefly while the FCU accepts the first local position hold.
                min_airborne_alt = min(min_airborne_alt, 0.05)
                min_airborne_grace = max(
                    min_airborne_grace,
                    self.takeoff_guided_handoff_time + 4.0,
                )

            if self._nav_altitude() < min_airborne_alt:
                if self._low_airborne_since is None:
                    self._low_airborne_since = now
                elif now - self._low_airborne_since > min_airborne_grace:
                    self.get_logger().error(
                        f"飞行中高度过低: alt={self._nav_altitude():.2f}m "
                        f"< {min_airborne_alt:.2f}m, 中止任务"
                    )
                    self.transition(State.ABORT)
                    return
            else:
                self._low_airborne_since = None

            if self.attitude_rpy is not None:
                roll, pitch, _ = self.attitude_rpy
                max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
                if max_rp > self.airborne_max_rp_deg:
                    if self._bad_attitude_since is None:
                        self._bad_attitude_since = now
                    elif now - self._bad_attitude_since > self.airborne_attitude_grace:
                        self.get_logger().error(
                            f"飞行中姿态过大: roll/pitch={max_rp:.1f}deg "
                            f"> {self.airborne_max_rp_deg:.1f}deg, 中止任务"
                        )
                        self.transition(State.ABORT)
                        return
                else:
                    self._bad_attitude_since = None

        skip_localization_check = (
            self.state == State.PUNCTURE_CONFIRM
            and self.simulate_balloon_puncture_on_detection
        )
        if time.time() < self._takeoff_localization_grace_until:
            skip_localization_check = True
        if (
            self.is_armed
            and self._nav_altitude() > self.takeoff_success_altitude
            and self.state in _mission_states
            and self.state not in (State.LANDING, State.SAFE_STOP, State.ABORT)
            and not self._guided_velocity_handoff_active()
            and not skip_localization_check
        ):
            if self._check_localization_consistency():
                return

        # B1修复: EKF健康监控带防重入标志, 避免ABORT死循环
        if self._nav_altitude() > 0.5 and not self.ekf_ok and self.is_armed:
            if not self._ekf_abort_triggered:
                self._ekf_abort_triggered = True
                self.get_logger().error(f"EKF异常! 高度={self._nav_altitude():.1f}m, 紧急降落")
                self.transition(State.ABORT)
            return

        # H5修复: ABORT→IDLE后EKF未恢复则阻止再次进入任务
        # 附加: 直接检查ekf_ok而非仅依赖_ekf_abort_triggered, 避免竞争条件
        if self.state == State.IDLE:
            if not self.ekf_ok:
                self._ekf_abort_triggered = True
            elif self._ekf_abort_triggered:
                self._ekf_abort_triggered = False  # EKF恢复后重置

        # B3修复: 飞控断连检测 (is_connected话题超过2秒未更新视为断连)
        if self.is_connected and time.time() - self._last_connected_time > 2.0:
            self.is_connected = False
            self.get_logger().error(
                f"飞控通信丢失! 最后心跳={self._last_connected_time:.0f}s前"
            )
            self.transition(State.ABORT)
            return

        if self.mission_start_time:
            elapsed = time.time() - self.mission_start_time
            if elapsed > self.max_mission_time:
                self.get_logger().error(f"任务超时({elapsed:.0f}s)!")
                self.transition(State.ABORT)
                return

        try:
            handlers = {
                State.IDLE: self._handle_idle,
                State.CHECK_SYSTEM: self._handle_check_system,
                State.WAIT_START: self._handle_wait_start,
                State.ARMING: self._handle_arming,
                State.WAIT_EXTERNAL_NAV: self._handle_wait_external_nav,
                State.TAKEOFF_RAMP: self._handle_takeoff_ramp,
                State.HOVER_STABLE: self._handle_hover_stable,
                State.WAYPOINT_NAV: self._handle_waypoint_nav,
                State.CODE_SEARCH: self._handle_code_search,
                State.WAIT_UGV_READY: self._handle_wait_ugv,
                State.BALLOON_SEARCH: self._handle_balloon_search,
                State.BALLOON_ALIGN: self._handle_balloon_align,
                State.PUNCTURE_DESCEND: self._handle_puncture_descend,
                State.PUNCTURE_CONFIRM: self._handle_puncture_confirm,
                State.RETURN_HOME_AFTER_STRIKE: self._handle_return_home_after_strike,
                State.LANDING: self._handle_landing,
                State.SAFE_STOP: self._handle_safe_stop,
                State.COMPLETE: self._handle_complete,
                State.ABORT: self._handle_abort,
            }
            handlers.get(self.state, lambda: None)()
        except Exception as e:
            self.get_logger().error(f"状态机异常: {e}")
            import traceback
            traceback.print_exc()
            self.transition(State.ABORT)

    # ===== 辅助方法 =====
    def _publish_rc(self, roll=1500, pitch=1500, throttle=None, yaw=1500):
        if throttle is None:
            throttle = self.hover_pwm
        joy = Joy()
        joy.axes = [float(roll), float(pitch), float(throttle), float(yaw)]
        self.rc_pub.publish(joy)

        # 每2秒打印一次RC输出值（油门确认日志）
        now = time.time()
        if now - self._last_rc_log_time >= 2.0:
            self._last_rc_log_time = now
            self.get_logger().info(
                f"[RC] roll={roll} pitch={pitch} throttle={throttle} yaw={yaw} "
                f"alt={self.current_alt:.3f}m mode={self.current_mode}"
            )

    def _publish_hover(self):
        self._publish_rc(1500, 1500, self.hover_pwm, 1500)

    def _takeoff_settle_throttle(self):
        if self.takeoff_settle_pwm > 0:
            return self.takeoff_settle_pwm
        return max(1300, min(1700, self.hover_pwm + self.takeoff_min_pwm_offset))

    def _takeoff_fallback_brake_throttle(self):
        if self.takeoff_fallback_brake_pwm > 0:
            return self.takeoff_fallback_brake_pwm
        if self.takeoff_settle_pwm > 0:
            return self.takeoff_settle_pwm
        legacy = self.hover_pwm + self.takeoff_min_pwm_offset
        return max(1150, min(1700, legacy))

    def _takeoff_fallback_ramp_throttle(self, nav_alt, climb_rate):
        now = time.time()
        if self._takeoff_fallback_ramp_pwm <= 0:
            self._takeoff_fallback_ramp_pwm = self.takeoff_fallback_ramp_start_pwm
            self._takeoff_fallback_next_ramp_time = now + self.takeoff_fallback_ramp_step_time

        if nav_alt >= self.takeoff_fallback_liftoff_altitude:
            self._takeoff_fallback_liftoff_detected = True

        brake_alt = max(
            self.takeoff_fallback_liftoff_altitude,
            min(self.takeoff_success_altitude * 0.80, self.takeoff_success_altitude - 0.03),
        )
        release_alt = max(0.04, min(self.takeoff_fallback_liftoff_altitude * 0.65, brake_alt - 0.05))
        if self._takeoff_fallback_liftoff_detected:
            release_alt = min(0.02, release_alt)
        should_brake = (
            nav_alt >= brake_alt
            or climb_rate >= self.takeoff_fallback_brake_climb_rate
            or (
                self._takeoff_fallback_liftoff_detected
                and climb_rate > -0.10
                and nav_alt >= self.takeoff_fallback_liftoff_altitude
            )
        )
        if should_brake:
            self._takeoff_fallback_braking = True
        elif (
            self._takeoff_fallback_braking
            and nav_alt <= release_alt
            and climb_rate <= -0.05
        ):
            self._takeoff_fallback_braking = False
            self._takeoff_fallback_next_ramp_time = now

        if self._takeoff_fallback_braking:
            return self._takeoff_fallback_brake_throttle()

        if (
            now >= self._takeoff_fallback_next_ramp_time
            and nav_alt < brake_alt
            and climb_rate < self.takeoff_fallback_brake_climb_rate
        ):
            self._takeoff_fallback_ramp_pwm = min(
                self.takeoff_fallback_ramp_max_pwm,
                self._takeoff_fallback_ramp_pwm + self.takeoff_fallback_ramp_step_pwm,
            )
            self._takeoff_fallback_next_ramp_time = now + self.takeoff_fallback_ramp_step_time

        return self._takeoff_fallback_ramp_pwm

    def _takeoff_fallback_altitude_rate(self, nav_alt):
        if self.localization_backend == 'external_nav' and not self.enable_sim_truth_monitor:
            return self._navigation_vertical_rate()
        now = time.time()
        if self._takeoff_fallback_last_alt is None or self._takeoff_fallback_last_time <= 0.0:
            self._takeoff_fallback_last_alt = nav_alt
            self._takeoff_fallback_last_time = now
            self._takeoff_fallback_climb_rate = 0.0
            return 0.0

        dt = now - self._takeoff_fallback_last_time
        if dt < 0.05:
            return self._takeoff_fallback_climb_rate

        rate = (nav_alt - self._takeoff_fallback_last_alt) / dt
        self._takeoff_fallback_last_alt = nav_alt
        self._takeoff_fallback_last_time = now
        if math.isfinite(rate):
            self._takeoff_fallback_climb_rate = rate
        return self._takeoff_fallback_climb_rate

    def _nav_altitude(self):
        """Return positive altitude above the local origin for mission decisions."""
        now = time.time()
        nav_position = self._navigation_position()
        if (
            self.enable_sim_truth_monitor
            and self.localization_backend == "external_nav"
            and self.sim_truth_position is not None
            and now - self._last_sim_truth_time <= self.sim_truth_stale_timeout
        ):
            return max(0.0, -float(self.sim_truth_position[2]))

        if nav_position is not None:
            altitude = -float(nav_position[2])
            # A fresh estimate at/below the takeoff reference means ground,
            # not a missing measurement. Falling back to noisy barometric
            # altitude here can falsely complete liftoff while still landed.
            if math.isfinite(altitude):
                return max(0.0, altitude)

        return max(0.0, float(self.current_alt))

    def _navigation_position(self):
        """Return the selected fresh local-NED position estimate."""
        now = time.time()
        if self.localization_backend == "external_nav":
            if (
                self.external_nav_position is not None
                and now - self._last_external_nav_pose_time <= self.external_nav_stale_timeout
            ):
                return self.external_nav_position
            return None
        if (
            self.local_position is not None
            and now - self._last_local_pos_time <= self.local_position_stale_timeout
        ):
            return self.local_position
        return None

    def _navigation_velocity(self):
        """Return selected local-NED velocity, using a position derivative if needed."""
        now = time.time()
        if self.localization_backend != "external_nav":
            if now - self._last_local_velocity_time <= self.local_position_stale_timeout:
                return self.local_velocity
            return (0.0, 0.0, 0.0)
        return self._external_nav_velocity

    def _navigation_speed_xy(self):
        vx, vy, _ = self._navigation_velocity()
        return math.hypot(vx, vy)

    def _navigation_vertical_rate(self):
        return -self._navigation_velocity()[2]

    def _navigation_is_ready(self):
        if self.localization_backend == "external_nav":
            return self.external_nav_ready and self._navigation_position() is not None
        return self.ekf_ok and self._navigation_position() is not None

    def _current_guided_xy(self):
        position = self._navigation_position()
        if position is not None:
            return float(position[0]), float(position[1])
        return 0.0, 0.0

    def _release_rc_override(self):
        self.rc_release_pub.publish(Bool(data=True))

    def _request_guided_mode(self):
        now = time.time()
        if now - self._last_guided_mode_req_time > 1.0:
            self._last_guided_mode_req_time = now
            self.mode_pub.publish(String(data="GUIDED"))
            self.get_logger().info(f"请求GUIDED模式 (当前={self.current_mode})")

    def _request_loiter_mode(self):
        now = time.time()
        if now - self._last_guided_mode_req_time > 1.0:
            self._last_guided_mode_req_time = now
            self.mode_pub.publish(String(data="LOITER"))
            self.get_logger().info(f"请求LOITER模式 (当前={self.current_mode})")

    def _request_mode_throttled(self, mode):
        now = time.time()
        if now - self._last_guided_mode_req_time > 1.0:
            self._last_guided_mode_req_time = now
            self.mode_pub.publish(String(data=mode))
            self.get_logger().info(f"请求{mode}模式 (当前={self.current_mode})")

    def _rc_landing_active(self):
        return self.landing_strategy in ("RC_DESCEND", "ALT_HOLD_RC", "STABILIZE_RC")

    def _landing_descent_throttle(self):
        if self.landing_descent_pwm > 0:
            return self.landing_descent_pwm
        return max(1300, min(1600, self.hover_pwm - 120))

    def _safe_stop_throttle(self):
        if self.safe_stop_descent_pwm > 0:
            return self.safe_stop_descent_pwm
        return self._landing_descent_throttle()

    def _publish_guided_position(self, x_north, y_east, altitude_m, label=""):
        """发布 GUIDED local NED 位置目标; altitude_m 为正的离地高度。"""
        now = time.time()
        if now - self._last_guided_target_time < self.guided_target_period:
            return
        self._last_guided_target_time = now

        target = Point()
        target.x = float(x_north)
        target.y = float(y_east)
        target.z = -float(altitude_m)  # MAV_FRAME_LOCAL_NED: z向下为正
        self.guided_pos_pub.publish(target)

        if now - getattr(self, "_guided_target_log_time", 0.0) > 3.0:
            self._guided_target_log_time = now
            self.get_logger().info(
                f"[GUIDED] target={label or 'HOLD'} "
                f"ned=({target.x:.2f},{target.y:.2f},{target.z:.2f}) "
                f"mode={self.current_mode}"
            )

    def _publish_guided_velocity(
        self,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0,
        label="",
        force=False,
        bypass_slew=False,
    ):
        """发布 GUIDED local NED 速度目标; z速度遵循NED, 向下为正。"""
        now = time.time()
        if not force and now - self._last_guided_target_time < self.guided_target_period:
            return
        self._last_guided_target_time = now

        vx = float(vx)
        vy = float(vy)
        vz = float(vz)
        prev = self._last_guided_velocity_cmd
        prev_time = self._last_guided_velocity_cmd_time
        if (
            not bypass_slew
            and prev is not None
            and now - prev_time <= max(1.0, self.guided_target_period * 5.0)
        ):
            dt = max(0.02, min(now - prev_time, 0.5))
            prev_vx, prev_vy, prev_vz = prev
            delta_xy = math.hypot(vx - prev_vx, vy - prev_vy)
            max_delta_xy = self.guided_velocity_slew_xy * dt
            if delta_xy > max_delta_xy > 0.0:
                scale = max_delta_xy / delta_xy
                vx = prev_vx + (vx - prev_vx) * scale
                vy = prev_vy + (vy - prev_vy) * scale
            max_delta_z = self.guided_velocity_slew_z * dt
            if abs(vz - prev_vz) > max_delta_z > 0.0:
                vz = prev_vz + math.copysign(max_delta_z, vz - prev_vz)
        self._last_guided_velocity_cmd = (vx, vy, vz)
        self._last_guided_velocity_cmd_time = now

        target = Twist()
        target.linear.x = vx
        target.linear.y = vy
        target.linear.z = vz
        target.angular.z = float(yaw_rate)
        self.guided_vel_pub.publish(target)

        if now - getattr(self, "_guided_velocity_log_time", 0.0) > 3.0:
            self._guided_velocity_log_time = now
            self.get_logger().info(
                f"[GUIDED速度] target={label or 'ZERO'} "
                f"vel_ned=({target.linear.x:.2f},{target.linear.y:.2f},{target.linear.z:.2f}) "
                f"yaw_rate={target.angular.z:.2f} mode={self.current_mode}"
                f"{' force' if force else ''}{' no_slew' if bypass_slew else ''}"
            )

    def _guided_ned_vz_for_altitude(self, target_altitude_m):
        """Return NED z velocity for altitude hold with stronger descent recovery."""
        nav_alt = self._nav_altitude()
        err_z = float(target_altitude_m) - nav_alt
        speed_cap = self.guided_nav_z_speed_max
        if err_z < -0.12 or (nav_alt > target_altitude_m and self._navigation_vertical_rate() > 0.05):
            speed_cap = max(speed_cap, self.guided_alt_recovery_z_speed_max)
        alt_speed = max(-speed_cap, min(speed_cap, self.guided_nav_k_z * err_z))
        return -alt_speed

    def _publish_guided_level_climb(self, climb_rate_mps):
        """发布 GUIDED 水平姿态爬升率, 仅用于专项诊断或手动启用的离地兜底。"""
        msg = Float32()
        msg.data = float(climb_rate_mps)
        self.guided_level_climb_pub.publish(msg)

    def _complete_guided_position_handoff(self, nav_alt, label):
        """Switch fallback liftoff back to normal GUIDED position hold."""
        hold_x, hold_y = self._current_guided_xy()
        min_hold_alt = max(
            self.takeoff_success_altitude,
            self.takeoff_guided_handoff_min_altitude,
            self.takeoff_fallback_liftoff_altitude,
        )
        if self._guided_hold_target is not None:
            _, _, previous_hold_alt, _ = self._guided_hold_target
            hold_alt = max(min_hold_alt, min(float(previous_hold_alt), self.target_altitude))
        else:
            hold_alt = min(self.target_altitude, max(nav_alt, min_hold_alt))

        # If fallback handoff happens while still settling vertically, hold the
        # current altitude first. HOVER_STABLE will climb to the task altitude
        # gently after XY/attitude have settled.
        if nav_alt > hold_alt:
            hold_alt = min(self.target_altitude, max(hold_alt, nav_alt))

        if nav_alt <= hold_alt + 0.12 and abs(self._navigation_vertical_rate()) <= self.hover_vertical_rate_max:
            hold_alt = min(self.target_altitude, max(nav_alt, hold_alt))

        self._guided_hold_target = (hold_x, hold_y, hold_alt, label)
        self._guided_handoff_done = True
        self._guided_handoff_level_since = 0.0
        self._guided_handoff_complete_time = time.time()
        self._last_hover_alt_update = time.time()
        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True
        self.get_logger().info(
            f"受限离地完成, 交回GUIDED位置悬停: "
            f"target=({hold_x:.2f},{hold_y:.2f},{hold_alt:.2f}m), "
            f"nav_alt={nav_alt:.2f}m, vz={self._navigation_vertical_rate():.2f}m/s, "
            f"mode={self.current_mode}"
        )

    def _publish_guided_handoff_hold(self, hold_x, hold_y, hold_alt, label):
        """Low-disturbance GUIDED hold during takeoff handoff."""
        if self.hover_guided_velocity_hold:
            self._publish_guided_hover_velocity(hold_x, hold_y, hold_alt, label)
            return
        self._publish_guided_position(hold_x, hold_y, hold_alt, f"{label}_HOLD")

    def _publish_guided_hold(self):
        """GUIDED内发布最近目标悬停; 非GUIDED仅保留应急悬停输出。"""
        if self.current_mode == "GUIDED" and self._guided_hold_target is not None:
            x, y, alt, label = self._guided_hold_target
            if self.state == State.HOVER_STABLE and self.hover_guided_velocity_hold:
                self._publish_guided_hover_velocity(x, y, alt, label)
                return
            if self.state == State.HOVER_STABLE:
                self._publish_guided_position(x, y, alt, f"HOLD_{label}")
                return
            if self.guided_nav_control_mode == "velocity":
                nav_position = self._navigation_position()
                if nav_position is not None:
                    cur_x, cur_y, _ = nav_position
                    err_x = x - cur_x
                    err_y = y - cur_y
                else:
                    err_x = 0.0
                    err_y = 0.0
                vx = self.guided_nav_k_xy * err_x
                vy = self.guided_nav_k_xy * err_y
                speed_xy = math.hypot(vx, vy)
                speed_cap = max(0.02, self.guided_hold_speed_max)
                if speed_xy > speed_cap:
                    scale = speed_cap / speed_xy
                    vx *= scale
                    vy *= scale
                vz = self._guided_ned_vz_for_altitude(alt)
                self._publish_guided_velocity(vx, vy, vz, 0.0, f"HOLD_{label}")
            else:
                self._publish_guided_position(x, y, alt, f"HOLD_{label}")
        elif not self.localization_mode:
            self._publish_hover()

    def _publish_guided_hover_velocity(self, hold_x, hold_y, hold_alt, label):
        """Use low-rate NED velocity hold during post-takeoff settling."""
        nav_position = self._navigation_position()
        if nav_position is None:
            self._publish_guided_velocity(0.0, 0.0, 0.0, 0.0, f"HOLD_{label}")
            return

        cur_x, cur_y, _ = nav_position
        err_x = hold_x - cur_x
        err_y = hold_y - cur_y
        cur_vx, cur_vy, _ = self._navigation_velocity()
        vx = 0.18 * err_x - 0.35 * cur_vx
        vy = 0.18 * err_y - 0.35 * cur_vy
        speed_xy = math.hypot(vx, vy)
        if speed_xy > self.hover_hold_speed_xy_max:
            scale = self.hover_hold_speed_xy_max / speed_xy
            vx *= scale
            vy *= scale

        nav_alt = self._nav_altitude()
        err_alt = hold_alt - nav_alt
        alt_rate = 0.16 * err_alt - 0.70 * self._navigation_vertical_rate()
        if alt_rate >= 0.0:
            alt_rate = min(self.hover_hold_z_speed_max, alt_rate)
        else:
            alt_rate = max(-self.hover_hold_descent_speed_max, alt_rate)
        vz = -alt_rate
        self._publish_guided_velocity(vx, vy, vz, 0.0, f"HOVER_{label}")

    def _compute_segment_follow_target(self, cur_x, cur_y, tgt_x, tgt_y, label):
        """Return a GUIDED target constrained to the active waypoint segment."""
        if self._wpt_index <= 0 or self._wpt_index >= len(self._wpt_waypoints):
            err_x = tgt_x - cur_x
            err_y = tgt_y - cur_y
            err_len = math.hypot(err_x, err_y)
            if err_len > 1.0e-4:
                return tgt_x, tgt_y, 0.0, 0.0, err_len, label, err_x / err_len, err_y / err_len
            return tgt_x, tgt_y, 0.0, 0.0, 0.0, label, 0.0, 0.0

        start_x, start_y, _, _ = self._wpt_waypoints[self._wpt_index - 1]
        seg_x = tgt_x - start_x
        seg_y = tgt_y - start_y
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1.0e-4:
            return tgt_x, tgt_y, 0.0, 0.0, 0.0, label, 0.0, 0.0

        ux = seg_x / seg_len
        uy = seg_y / seg_len
        rel_x = cur_x - start_x
        rel_y = cur_y - start_y
        along = rel_x * ux + rel_y * uy
        cross = rel_x * (-uy) + rel_y * ux
        along_clamped = max(0.0, min(seg_len, along))
        progress_key = self._wpt_index
        now = time.time()
        prev_along = self._guided_segment_progress.get(progress_key, 0.0)
        if abs(cross) <= self.guided_nav_cross_track_stop and along_clamped > prev_along:
            last_time = self._guided_segment_progress_time.get(progress_key, now)
            dt = max(0.02, min(now - last_time, 0.5))
            max_delta = max(0.01, self.guided_nav_speed_max * dt * 1.2)
            prev_along = min(along_clamped, prev_along + max_delta)
            self._guided_segment_progress[progress_key] = prev_along
            self._guided_segment_progress_time[progress_key] = now
            along_clamped = prev_along
        else:
            self._guided_segment_progress_time[progress_key] = now
            along_clamped = prev_along

        lookahead = self.guided_path_lookahead
        if self.guided_step_distance > 0.05:
            lookahead = min(lookahead, self.guided_step_distance)
        if abs(cross) > self.guided_nav_cross_track_stop:
            lookahead = 0.0
        elif abs(cross) > self.guided_path_cross_track_warn:
            lookahead = min(lookahead, self.guided_path_recovery_lookahead)
        goal_along = min(seg_len, along_clamped + lookahead)

        cmd_x = start_x + ux * goal_along
        cmd_y = start_y + uy * goal_along
        return cmd_x, cmd_y, cross, along_clamped, seg_len, f"{label}_PATH", ux, uy

    def _position_nav_brake_needed(self):
        """Return True when position-mode navigation should pause XY progress."""
        now = time.time()
        attitude_brake = False
        if self.attitude_rpy is not None and now - self._last_attitude_time <= 1.0:
            roll, pitch, _ = self.attitude_rpy
            max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            attitude_brake = max_rp > self.guided_nav_attitude_brake_rp_deg
            if attitude_brake and now - self._last_position_brake_log > 1.0:
                self._last_position_brake_log = now
                self.get_logger().warn(
                    f"WAYPOINT_NAV: 姿态{max_rp:.1f}deg偏大, 位置目标暂停推进"
                )

        speed_brake = self._navigation_speed_xy() > self.guided_position_brake_speed_xy
        if speed_brake and now - self._last_position_brake_log > 1.0:
            self._last_position_brake_log = now
            self.get_logger().warn(
                f"WAYPOINT_NAV: XY速度{self._navigation_speed_xy():.2f}m/s偏大, "
                f"位置目标暂停推进"
            )

        if self._guided_position_brake_target is not None:
            return (
                attitude_brake
                or self._navigation_speed_xy() > self.guided_position_brake_resume_speed_xy
            )
        return attitude_brake or speed_brake

    def _limit_guided_position_step(self, cur_x, cur_y, cmd_x, cmd_y, label):
        """Limit each GUIDED position update to a short local step."""
        max_step = max(0.03, self.guided_step_distance)
        dx = cmd_x - cur_x
        dy = cmd_y - cur_y
        dist = math.hypot(dx, dy)
        if dist <= max_step or dist <= 1.0e-4:
            return cmd_x, cmd_y, label
        scale = max_step / dist
        return cur_x + dx * scale, cur_y + dy * scale, f"{label}_STEP"

    def _limit_xy_step(self, cur_x, cur_y, target_x, target_y, max_step, label):
        """Limit an XY target to a caller supplied step distance."""
        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)
        if dist <= max_step or dist <= 1.0e-4:
            return target_x, target_y, label
        scale = max_step / dist
        return cur_x + dx * scale, cur_y + dy * scale, f"{label}_STEP"

    def _guided_velocity_handoff_active(self):
        """Return True while RC/attitude liftoff is being handed back to GUIDED velocity hold."""
        return (
            self.state == State.HOVER_STABLE
            and (self._takeoff_use_rc_fallback or self._takeoff_use_guided_attitude_fallback)
            and not self._guided_handoff_done
        )

    def _start_waypoint_nav_after_takeoff(self, reason):
        """Enter the slow GUIDED waypoint route once low-altitude takeoff is established."""
        if len(self._wpt_waypoints) <= 1:
            self.get_logger().error("起飞后无法进入航点: 航点列表不足")
            self.transition(State.LANDING)
            return

        self._wpt_index = 1
        self._wpt_deadline = time.time() + self.waypoint_timeout
        self._guided_hold_target = self._wpt_waypoints[self._wpt_index]
        self._waypoint_arrival_stable_since = None
        self._guided_handoff_done = True
        self._guided_handoff_level_since = 0.0
        self._guided_handoff_complete_time = time.time()
        if self.takeoff_localization_grace_time > 0.0:
            self._takeoff_localization_grace_until = (
                self._guided_handoff_complete_time + self.takeoff_localization_grace_time
            )
        self._localization_fault_since = None
        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True
        self.get_logger().info(
            f"{reason}: 跳过HOVER_STABLE, 直接进入低速GUIDED航点 "
            f"[{self._wpt_index}] {self._wpt_waypoints[self._wpt_index][3]}, "
            f"定位一致性宽限={self.takeoff_localization_grace_time:.1f}s"
        )
        self.transition(State.WAYPOINT_NAV)

    def _publish_waypoints(self):
        """发布航点标记到 RViz (NED XY→map XY; z is positive altitude)."""
        markers = []
        short_labels = {
            "APPROACH_FRAME": "FRAME",
            "SCAN_ZONE": "SCAN",
            "RETURN_TAKEOFF_ZONE": "RETURN",
            "UGV_STOP_APPROACH": "APPROACH",
            "STRIKE_ZONE": "STRIKE",
            "BALLOON_ALT": "BALLOON",
        }
        # Several competition stages intentionally share an XY point. Offset
        # their top-down labels so the default RViz view stays readable.
        label_offsets = (
            (-0.08, 0.28),
            (0.00, 0.25),
            (0.00, -0.25),
            (0.00, -0.25),
            (0.12, 0.25),
            (-0.08, 0.25),
            (0.08, -0.25),
            (-0.08, -0.28),
        )
        for i, (xn, ye, z, label) in enumerate(self._wpt_waypoints):
            m = Marker()
            m.header.frame_id = self.waypoint_marker_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # The route stores positive altitude, while uav_map uses Z up.
            m.pose.position.x = xn
            m.pose.position.y = -ye
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12

            if i == self._wpt_index:
                # 当前目标: 绿色
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif i < self._wpt_index:
                # 已到达: 灰色
                m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
            else:
                # 未来: 黄色
                m.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.7)
            markers.append(m)

            # 标签文字
            text = Marker()
            text.header.frame_id = self.waypoint_marker_frame
            text.header.stamp = m.header.stamp
            text.ns = "waypoint_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            offset_x, offset_y = label_offsets[i % len(label_offsets)]
            text.pose.position.x = xn + offset_x
            text.pose.position.y = -ye + offset_y
            text.pose.position.z = z + 0.18
            text.scale.z = 0.10
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            text.text = f"WP{i} {short_labels.get(label, label)}"
            markers.append(text)

        self.waypoint_pub.publish(MarkerArray(markers=markers))

    def _elapsed(self):
        return time.time() - self.state_start_time if self.state_start_time else 0.0

    def _publish_strike_permission(self, allowed: bool):
        self.strike_permission = bool(allowed)
        self.strike_perm_pub.publish(Bool(data=self.strike_permission))

    def _publish_balloon_punctured(self, punctured: bool):
        self.balloon_punctured = bool(punctured)
        self.balloon_puncture_pub.publish(Bool(data=self.balloon_punctured))

    def _publish_ugv_start_once(self):
        if self._ugv_start_sent:
            return
        self._ugv_start_sent = True
        self.ugv_start_pub.publish(Bool(data=True))
        self.get_logger().info("规则进度: 已发布UGV出发许可 /mission/ugv_start=True")

    def _rules_allow_strike(self):
        code_ok = self.code_task_done or not self.require_code_success
        ugv_ok = self.ugv_in_strike_zone or not self.require_ugv_ready_for_strike
        if self.allow_strike_without_ugv:
            ugv_ok = True
        return (
            code_ok
            and self.return_frame_done
            and self.return_pillars_done
            and self.uav_back_in_strike_zone
            and ugv_ok
        )

    def _deny_balloon_stage(self, reason: str):
        self._publish_strike_permission(False)
        self.get_logger().error(f"规则门禁拒绝进入扎气球阶段: {reason}")
        if not self.code_task_done and self.require_code_success:
            self.transition(State.LANDING)
        elif not self.uav_back_in_strike_zone:
            self.transition(State.LANDING)
        else:
            self.transition(State.WAIT_UGV_READY)

    # ===== IDLE =====
    def _handle_idle(self):
        if self._elapsed() > 2.0:
            self.transition(State.CHECK_SYSTEM)

    # ===== CHECK_SYSTEM =====
    def _handle_check_system(self):
        elapsed = self._elapsed()

        if not self.is_connected:
            if self.has_parameter("connection_timeout"):
                connect_timeout = float(self.get_parameter("connection_timeout").value)
            else:
                connect_timeout = 10.0
            if elapsed > connect_timeout:
                self.get_logger().error(f"飞控连接超时({connect_timeout:.0f}s)!")
                self.transition(State.ABORT)
            return

        # 电池电压检查 (仿真中 min_battery_voltage=0.0 跳过)
        min_batt = float(self.get_parameter("min_battery_voltage").value)
        if self.battery_voltage > 0 and min_batt > 0 and self.battery_voltage < min_batt:
            self.get_logger().error(f"电池电压过低: {self.battery_voltage:.1f}V")
            self.transition(State.ABORT)
            return

        # H4修复: 预解锁检查 — 必须条件不满足则循环等待
        checks = []
        checks.append(f"飞控={'已连接' if self.is_connected else '未连接'}")
        checks.append(f"电池={'{:.1f}V'.format(self.battery_voltage) if self.battery_voltage > 0 else '...'}")
        checks.append(f"EKF={'OK' if self.ekf_ok else '...'}")
        checks.append(
            f"导航({self.localization_backend})={'OK' if self._navigation_is_ready() else '...'}"
        )
        checks.append(f"模式={self.current_mode}")
        home_ready = not self.native_guided_takeoff or self.fcu_home_ready
        if self.native_guided_takeoff:
            checks.append(f"HOME={'OK' if home_ready else '...'}")
        if self.is_connected and self.ekf_ok and self._navigation_is_ready() and home_ready:
            self.get_logger().info(
                f"系统自检通过: {' | '.join(checks)}"
            )
            self.transition(State.WAIT_START)
        elif elapsed > 60.0:
            self.get_logger().error(f"系统自检超时: {' | '.join(checks)}")
            self.transition(State.ABORT)
        else:
            if elapsed > 0.5 and elapsed - getattr(self, '_last_check_log', -5.0) > 5.0:
                self._last_check_log = elapsed
                self.get_logger().info(f"等待系统就绪({elapsed:.0f}s): {' | '.join(checks)}")

    # ===== WAIT_START =====
    def _handle_wait_start(self):
        if self.mission_started:
            self.mission_start_time = time.time()
            self.get_logger().info("比赛任务开始!")
            self._publish_ugv_start_once()
            self.get_logger().info("协同策略: UAV与UGV并行出发, 先到最终扎球点的一方等待")
            self.transition(State.ARMING)  # 先在STABILIZE模式下解锁
        elif not self.ignore_mission_start and self._elapsed() > self.max_start_time:
            self.get_logger().error(f"等待比赛开始超时({self.max_start_time:.0f}s)!")
            self.transition(State.ABORT)

    # ===== WAIT_EXTERNAL_NAV =====
    def _handle_wait_external_nav(self):
        if self._navigation_is_ready():
            self.get_logger().info(f"{self.localization_backend}定位就绪!")
            self.transition(State.TAKEOFF_RAMP)
        elif self._elapsed() > 5.0:
            self.get_logger().error(
                f"{self.localization_backend}定位超时({self._elapsed():.1f}s)! "
                "无法定位, 中止任务"
            )
            self.transition(State.ABORT)

    # ===== ARMING =====
    def _handle_arming(self):
        elapsed = self._elapsed()
        # STABILIZE模式最宽松, 无GPS/位置要求即可解锁
        if self.current_mode != "STABILIZE":
            # 限流: 每2秒发一次模式切换，防止阻塞Bridge执行器 (C4修复)
            now = time.time()
            if now - self._last_mode_req_time > 2.0:
                self._last_mode_req_time = now
                self.mode_pub.publish(String(data="STABILIZE"))
                self.get_logger().info(
                    f"ARMING: 请求STABILIZE模式 (当前={self.current_mode}, "
                    f"已耗时={elapsed:.1f}s, armed={self.is_armed})"
                )
            else:
                # 限流跳过，每5秒输出一次状态避免日志刷屏
                if now - getattr(self, '_last_arming_skip_log', 0.0) > 5.0:
                    self._last_arming_skip_log = now
                    self.get_logger().info(
                        f"ARMING: 等待模式切换 (当前={self.current_mode}, "
                        f"已耗时={elapsed:.1f}s, 下次模式请求={2.0 - (now - self._last_mode_req_time):.1f}s后)"
                    )
            return

        # 当前模式已是STABILIZE，持续发送解锁指令
        self.arm_pub.publish(Bool(data=True))
        if self.is_armed:
            self.get_logger().info(
                f"ARMING: 飞控已解锁! (模式={self.current_mode}, 耗时={elapsed:.1f}s)"
            )
            self.transition(State.WAIT_EXTERNAL_NAV)
        elif elapsed > float(self.get_parameter("arming_timeout").value):
            self.get_logger().error(
                f"ARMING: 解锁超时! (耗时={elapsed:.1f}s, "
                f"模式={self.current_mode}, armed={self.is_armed}, "
                f"ekf={self.ekf_ok}, batt={self.battery_voltage:.1f}V)"
            )
            self.transition(State.ABORT)
        else:
            # 每3秒报告一次ARMING进度
            if time.time() - getattr(self, '_last_arming_progress_log', 0.0) > 3.0:
                self._last_arming_progress_log = time.time()
                self.get_logger().info(
                    f"ARMING: 等待解锁... (耗时={elapsed:.1f}s, "
                    f"模式={self.current_mode}, armed={self.is_armed})"
                )

    # ===== TAKEOFF_RAMP =====
    def _handle_takeoff_ramp(self):
        self._handle_guided_takeoff()

    def _handle_guided_takeoff(self):
        """GUIDED takeoff for the ExternalNav mainline."""
        elapsed = self._elapsed()
        nav_alt = self._nav_altitude()
        target_alt = max(self.target_altitude, self.takeoff_success_altitude)

        if self.takeoff_start_alt is None:
            self.takeoff_start_alt = nav_alt
            if self.takeoff_hold_fixed_origin:
                self._takeoff_xy = (0.0, 0.0)
            else:
                self._takeoff_xy = self._current_guided_xy()
            self._guided_hold_target = (self._takeoff_xy[0], self._takeoff_xy[1], target_alt, "GUIDED_TAKEOFF")
            self._last_guided_target_time = 0.0
            self.get_logger().info(
                f"GUIDED_TAKEOFF 开始: 初始高度={nav_alt:.3f}m, "
                f"目标=({self._takeoff_xy[0]:.2f},{self._takeoff_xy[1]:.2f},{target_alt:.2f}m), "
                f"模式={self.current_mode}, 已解锁={self.is_armed}"
            )
            if self.prefer_takeoff_rc_fallback:
                if not self.allow_takeoff_rc_fallback:
                    self.get_logger().error(
                        "prefer_takeoff_rc_fallback=true 但 allow_takeoff_rc_fallback=false"
                    )
                    self.transition(State.ABORT)
                    return
                self._takeoff_use_rc_fallback = True
                self._takeoff_fallback_start_time = time.time()
                self.get_logger().info(
                    f"仿真起飞策略: 直接使用受限{self.takeoff_fallback_mode}"
                    "真实电机离地, 不发送当前无GPS配置不支持的GUIDED TAKEOFF"
                )

        if self._takeoff_use_guided_attitude_fallback:
            self._handle_takeoff_guided_attitude_fallback(nav_alt, target_alt)
            return

        if self._takeoff_use_rc_fallback:
            self._handle_takeoff_rc_fallback(nav_alt, target_alt)
            return

        if self.current_mode != "GUIDED":
            self._release_rc_override()
            self._request_guided_mode()
            self._publish_guided_position(
                self._takeoff_xy[0], self._takeoff_xy[1], target_alt, "GUIDED_TAKEOFF_WAIT_MODE"
            )
            if elapsed > self.guided_mode_timeout:
                self.get_logger().error(f"GUIDED_TAKEOFF: GUIDED切换超时(当前={self.current_mode})")
                self.transition(State.ABORT)
            return

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True

        # The point-cloud profile uses a streamed LOCAL_NED position ramp.
        # Sending NAV_TAKEOFF as well lets ArduPilot's native submode keep
        # climbing while ICP is still warming up, which can outrun the 10 Hz
        # Mid-360 external-nav stream.  Reserve NAV_TAKEOFF for the explicit
        # native-guided profile and let the ramp own the vertical target here.
        if self.native_guided_takeoff and not self._takeoff_command_sent:
            self.takeoff_pub.publish(Float32(data=float(target_alt)))
            self._takeoff_command_sent = True
            self.get_logger().info(f"已发送GUIDED起飞命令: {target_alt:.2f}m")

        if self.native_guided_takeoff:
            # A position target would cancel ArduPilot's native takeoff
            # submode. Let its altitude controller finish before handover.
            if nav_alt >= target_alt - 0.10 and abs(self._navigation_vertical_rate()) < 0.20:
                self._guided_hold_target = (*self._takeoff_xy, target_alt, 'NATIVE_GUIDED_TAKEOFF')
                self._guided_handoff_done = True
                self._guided_handoff_complete_time = time.time()
                self.get_logger().info(f'原生GUIDED起飞完成: 高度={nav_alt:.2f}m')
                self.transition(State.HOVER_STABLE)
            elif elapsed > 45.:
                self.get_logger().error('原生GUIDED起飞未达到目标高度，中止任务')
                self.transition(State.ABORT)
            return

        if (
            (self.use_guided_attitude_takeoff_fallback or self.allow_takeoff_rc_fallback)
            and elapsed > 3.0
            and nav_alt < max(0.08, self.takeoff_start_alt + 0.08)
        ):
            self._takeoff_fallback_start_time = time.time()
            self._rc_released_for_guided = False
            self._takeoff_fallback_last_alt = None
            self._takeoff_fallback_last_time = 0.0
            self._takeoff_fallback_climb_rate = 0.0
            self._takeoff_fallback_braking = False
            self._takeoff_fallback_ramp_pwm = 0
            self._takeoff_fallback_next_ramp_time = 0.0
            self._takeoff_fallback_liftoff_detected = False
            self._takeoff_fallback_handoff_stable_since = 0.0
            self._takeoff_fallback_high_rate_since = 0.0
            if self.use_guided_attitude_takeoff_fallback:
                self._takeoff_use_guided_attitude_fallback = True
                self.get_logger().warn(
                    "GUIDED起飞无高度响应, 启用手动配置的GUIDED水平姿态爬升兜底; "
                    "达到低空验收高度后立即交回GUIDED位置悬停"
                )
                self._handle_takeoff_guided_attitude_fallback(nav_alt, target_alt)
            else:
                if not self.allow_takeoff_rc_fallback:
                    self.get_logger().error(
                        f"GUIDED起飞无高度响应且未允许受限{self.takeoff_fallback_mode}离地兜底, 中止"
                    )
                    self.transition(State.ABORT)
                    return
                self._takeoff_use_rc_fallback = True
                self.get_logger().warn(
                    f"GUIDED起飞无高度响应, 启用受限{self.takeoff_fallback_mode}油门离地兜底; "
                    "达到低空验收高度后立即回到GUIDED"
                )
                self._handle_takeoff_rc_fallback(nav_alt, target_alt)
            return

        progress = min(elapsed / max(self.takeoff_ramp_time, 0.1), 1.0)
        cmd_alt = max(
            self.takeoff_success_altitude,
            self.takeoff_start_alt + (target_alt - self.takeoff_start_alt) * progress,
        )
        self._guided_hold_target = (self._takeoff_xy[0], self._takeoff_xy[1], cmd_alt, "GUIDED_TAKEOFF")
        self._publish_guided_position(self._takeoff_xy[0], self._takeoff_xy[1], cmd_alt, "GUIDED_TAKEOFF")

        now = time.time()
        if now - self._last_alt_log_time >= 2.0:
            self._last_alt_log_time = now
            self.get_logger().info(
                f"[GUIDED_TAKEOFF_DEBUG] 高度={nav_alt:.3f}m, 指令高度={cmd_alt:.3f}m, "
                f"进度={progress:.2f}, 已耗时={elapsed:.1f}s, 模式={self.current_mode}"
            )

        if nav_alt >= self.takeoff_success_altitude:
            self.get_logger().info(f"GUIDED起飞成功! 到达高度 {nav_alt:.2f}m")
            self._guided_hold_target = (self._takeoff_xy[0], self._takeoff_xy[1], target_alt, "TAKEOFF_HOLD")
            self.transition(State.HOVER_STABLE)
        elif elapsed > max(12.0, self.takeoff_ramp_time + 6.0):
            self.get_logger().error(
                f"GUIDED起飞超时! 高度={nav_alt:.3f}m, 指令高度={cmd_alt:.3f}m"
            )
            self.transition(State.ABORT)

    def _handle_takeoff_guided_attitude_fallback(self, nav_alt, target_alt):
        """Use GUIDED SET_ATTITUDE_TARGET level climb before any RC liftoff fallback."""
        now = time.time()
        fallback_elapsed = now - self._takeoff_fallback_start_time

        if self.current_mode != "GUIDED":
            self._release_rc_override()
            self._request_guided_mode()
            return

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True

        vertical_rate = self._navigation_vertical_rate()
        slow_band = min(0.24, max(0.10, self.takeoff_success_altitude * 0.45))
        slow_start_alt = max(0.0, self.takeoff_success_altitude - slow_band)
        max_liftoff_climb = max(0.02, min(0.10, self.takeoff_guided_fallback_climb_rate))
        max_handoff_climb = max(0.0, min(0.04, self.takeoff_guided_handoff_climb_rate))

        if nav_alt < slow_start_alt and vertical_rate < 0.12:
            climb_rate = max_liftoff_climb
        else:
            alt_error = self.takeoff_success_altitude - nav_alt
            climb_rate = 0.25 * alt_error - 1.35 * vertical_rate
            upper_limit = max_handoff_climb if nav_alt >= slow_start_alt else max_liftoff_climb
            handoff_alt = max(self.takeoff_success_altitude, self.takeoff_guided_handoff_min_altitude)
            if nav_alt >= handoff_alt:
                upper_limit = 0.0
            climb_rate = max(-2.60, min(upper_limit, climb_rate))

        if vertical_rate > 0.25:
            climb_rate = min(climb_rate, -1.35 * vertical_rate)
        if nav_alt > self.takeoff_success_altitude + 0.25:
            climb_rate = min(climb_rate, -2.20)
        self._publish_guided_level_climb(climb_rate)

        if now - self._last_alt_log_time >= 1.0:
            self._last_alt_log_time = now
            self.get_logger().info(
                f"[TAKEOFF_GUIDED_ATT_FALLBACK] 高度={nav_alt:.3f}m, "
                f"vz={vertical_rate:.2f}m/s, climb_cmd={climb_rate:.2f}m/s, "
                f"模式={self.current_mode}"
            )

        handoff_alt = max(self.takeoff_success_altitude, self.takeoff_guided_handoff_min_altitude)
        handoff_rate_ready = abs(vertical_rate) <= self.takeoff_fallback_handoff_vertical_rate_max
        handoff_attitude_ready = False
        handoff_attitude_reason = "姿态超时"
        if self.attitude_rpy is not None and now - self._last_attitude_time <= 1.0:
            roll, pitch, _ = self.attitude_rpy
            max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            handoff_attitude_ready = max_rp <= self.takeoff_guided_handoff_max_rp_deg
            handoff_attitude_reason = (
                f"姿态{max_rp:.1f}/{self.takeoff_guided_handoff_max_rp_deg:.1f}deg"
            )
        if nav_alt >= handoff_alt and not (handoff_rate_ready and handoff_attitude_ready):
            self._publish_guided_level_climb(-0.20)
            if now - self._last_guided_handoff_log >= 1.0:
                self._last_guided_handoff_log = now
                wait_reasons = []
                if not handoff_rate_ready:
                    wait_reasons.append(
                        f"vz={vertical_rate:.2f}/{self.takeoff_fallback_handoff_vertical_rate_max:.2f}m/s"
                    )
                if not handoff_attitude_ready:
                    wait_reasons.append(handoff_attitude_reason)
                self.get_logger().info(
                    f"GUIDED水平姿态已离地但交权条件未满足, 继续低推力刹车: "
                    f"高度={nav_alt:.2f}/{handoff_alt:.2f}m, vz={vertical_rate:.2f}m/s, "
                    f"等待={'; '.join(wait_reasons)}"
                )
            if nav_alt > self.takeoff_fallback_max_altitude:
                self.get_logger().error(
                    f"GUIDED水平姿态离地高度过冲: {nav_alt:.2f}m > "
                    f"{self.takeoff_fallback_max_altitude:.2f}m, 进入降落"
                )
                self.transition(State.LANDING)
            return

        if nav_alt >= handoff_alt:
            self._publish_guided_level_climb(-0.50)
            hold_x, hold_y = self._current_guided_xy()
            hold_alt = min(target_alt, max(nav_alt, self.takeoff_success_altitude))
            self.get_logger().info(
                f"GUIDED水平姿态离地完成: 高度={nav_alt:.2f}m, "
                f"垂向速度={vertical_rate:.2f}m/s, "
                f"垂向速度已收敛, 交回位置悬停({hold_x:.2f},{hold_y:.2f},{hold_alt:.2f}m)"
            )
            self._guided_hold_target = (hold_x, hold_y, hold_alt, "TAKEOFF_ATTITUDE_HANDOFF")
            self._guided_handoff_done = False
            self._guided_handoff_level_since = 0.0
            self._last_guided_handoff_log = 0.0
            if self.skip_hover_stable_after_takeoff:
                self._start_waypoint_nav_after_takeoff("GUIDED水平姿态离地完成")
                return
            self.transition(State.HOVER_STABLE)
            return

        if fallback_elapsed > max(4.0, self.takeoff_guided_fallback_timeout):
            response_alt = max(0.04, min(0.10, self.takeoff_success_altitude * 0.45))
            start_alt = self.takeoff_start_alt if self.takeoff_start_alt is not None else 0.0
            has_height_response = (
                nav_alt >= start_alt + response_alt
                or nav_alt >= self.takeoff_success_altitude * 0.45
                or vertical_rate >= 0.015
            )
            if has_height_response:
                if now - self._last_guided_handoff_log >= 1.5:
                    self._last_guided_handoff_log = now
                    self.get_logger().info(
                        "GUIDED水平姿态爬升已有高度响应, 继续保持GUIDED离地, "
                        f"不降级{self.takeoff_fallback_mode}: "
                        f"高度={nav_alt:.2f}m, vz={vertical_rate:.2f}m/s"
                    )
                return
            if self.allow_takeoff_rc_fallback:
                self._takeoff_use_guided_attitude_fallback = False
                self._takeoff_use_rc_fallback = True
                self._takeoff_fallback_start_time = now
                self._rc_released_for_guided = False
                self._takeoff_fallback_last_alt = None
                self._takeoff_fallback_last_time = 0.0
                self._takeoff_fallback_climb_rate = 0.0
                self._takeoff_fallback_braking = False
                self._takeoff_fallback_ramp_pwm = 0
                self._takeoff_fallback_next_ramp_time = 0.0
                self._takeoff_fallback_liftoff_detected = False
                self._takeoff_fallback_handoff_stable_since = 0.0
                self._takeoff_fallback_high_rate_since = 0.0
                self.get_logger().warn(
                    "GUIDED水平姿态爬升无高度响应, 降级到受限"
                    f"{self.takeoff_fallback_mode}油门离地兜底"
                )
                self._handle_takeoff_rc_fallback(nav_alt, target_alt)
            else:
                self.get_logger().error(
                    f"GUIDED水平姿态离地超时! 高度={nav_alt:.3f}m"
                )
                self.transition(State.ABORT)

    def _handle_takeoff_rc_fallback(self, nav_alt, target_alt):
        """Bounded liftoff fallback only; navigation remains GUIDED."""
        now = time.time()
        fallback_elapsed = now - self._takeoff_fallback_start_time
        climb_rate = self._takeoff_fallback_altitude_rate(nav_alt)
        throttle = self._takeoff_fallback_ramp_throttle(nav_alt, climb_rate)
        brake_pwm = self._takeoff_fallback_brake_throttle()
        fallback_mode = self.takeoff_fallback_mode
        max_alt = max(self.takeoff_success_altitude + 0.10, self.takeoff_fallback_max_altitude)

        if nav_alt >= max_alt:
            self._takeoff_fallback_braking = True
            self._publish_rc(throttle=brake_pwm)
            self.get_logger().error(
                f"受限{fallback_mode}离地高度过冲, 立即转入降落: "
                f"高度={nav_alt:.3f}m/{max_alt:.2f}m, 爬升率={climb_rate:.2f}m/s"
            )
            self.transition(State.LANDING)
            return

        if self.current_mode != fallback_mode:
            self._request_mode_throttled(fallback_mode)
            self._publish_rc(throttle=throttle)
            return

        self._publish_rc(throttle=throttle)

        if now - self._last_alt_log_time >= 1.0:
            self._last_alt_log_time = now
            self.get_logger().info(
                f"[TAKEOFF_RC_FALLBACK] 高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s, "
                f"油门={throttle}, ramp={self._takeoff_fallback_ramp_pwm}, "
                f"braking={self._takeoff_fallback_braking}, liftoff={self._takeoff_fallback_liftoff_detected}, "
                f"模式={self.current_mode}"
            )

        # Do not hand RC control back to GUIDED while the vehicle is merely
        # above the liftoff detector.  The first local-position hold can dip
        # briefly; require the configured low-altitude handoff target before
        # releasing the proven throttle hold.
        handoff_altitude = max(
            self.takeoff_fallback_liftoff_altitude,
            self.takeoff_guided_handoff_min_altitude,
        )
        handoff_alt_ready = (
            self._takeoff_fallback_liftoff_detected
            and nav_alt >= handoff_altitude
        )
        handoff_rate_ready = (
            abs(climb_rate) <= self.takeoff_fallback_handoff_vertical_rate_max
        )
        handoff_ready = False
        if handoff_alt_ready and climb_rate > self.takeoff_fallback_handoff_vertical_rate_max:
            self._takeoff_fallback_braking = True
            throttle = brake_pwm
            self._publish_rc(throttle=throttle)
            if now - self._last_guided_handoff_log >= 1.0:
                self._last_guided_handoff_log = now
                self.get_logger().info(
                    f"受限{fallback_mode}已离地但爬升率未收敛, 继续刹车等待GUIDED交权: "
                    f"高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s, "
                    f"目标<={self.takeoff_fallback_handoff_vertical_rate_max:.2f}m/s, brake_pwm={brake_pwm}"
                )
            return

        if handoff_alt_ready and handoff_rate_ready:
            handoff_reasons = self._takeoff_rc_fallback_handoff_reasons(nav_alt, climb_rate)
            if handoff_reasons:
                self._takeoff_fallback_braking = True
                self._takeoff_fallback_handoff_stable_since = 0.0
                throttle = brake_pwm
                self._publish_rc(throttle=throttle)
                if now - self._last_guided_handoff_log >= 1.0:
                    self._last_guided_handoff_log = now
                    self.get_logger().info(
                        f"受限{fallback_mode}离地后等待稳定再GUIDED交权: "
                        f"{'; '.join(handoff_reasons)} "
                        f"高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s, brake_pwm={brake_pwm}"
                    )
                return
            if self._takeoff_fallback_handoff_stable_since <= 0.0:
                self._takeoff_fallback_handoff_stable_since = now
                self.get_logger().info(
                    f"受限{fallback_mode}离地垂向速度已收敛, 准备GUIDED交权: "
                    f"高度={nav_alt:.2f}m, 爬升率={climb_rate:.2f}m/s"
                )
            throttle = self._takeoff_settle_throttle()
            self._publish_rc(throttle=throttle)
            handoff_ready = (
                now - self._takeoff_fallback_handoff_stable_since
                >= self.takeoff_fallback_handoff_stable_time
            )
        else:
            self._takeoff_fallback_handoff_stable_since = 0.0

        if handoff_ready:
            cur_x, cur_y = self._current_guided_xy()
            hold_alt = min(
                target_alt,
                max(
                    nav_alt,
                    self.takeoff_success_altitude,
                    self.takeoff_guided_handoff_min_altitude,
                    self.takeoff_fallback_liftoff_altitude,
                ),
            )
            self._takeoff_fallback_braking = True
            self._publish_rc(throttle=self._takeoff_settle_throttle())
            self.get_logger().info(
                f"受限{fallback_mode}离地完成: 高度={nav_alt:.2f}m, 爬升率={climb_rate:.2f}m/s, "
                f"先切回GUIDED当前位置保持交权, 再进入当前位置悬停({cur_x:.2f},{cur_y:.2f},{hold_alt:.2f}m)"
            )
            self._takeoff_settle_until = 0.0
            self._guided_hold_target = (cur_x, cur_y, hold_alt, "TAKEOFF_HANDOFF")
            self._guided_handoff_done = False
            self._guided_handoff_level_since = 0.0
            self._rc_released_for_guided = False
            self.transition(State.HOVER_STABLE)
            return

        climb_abort = (
            nav_alt >= self.takeoff_fallback_liftoff_altitude
            and climb_rate >= self.takeoff_fallback_abort_climb_rate
        )
        if climb_abort:
            self._takeoff_fallback_braking = True
            self._publish_rc(throttle=brake_pwm)
            if self._takeoff_fallback_high_rate_since <= 0.0:
                self._takeoff_fallback_high_rate_since = now
                self.get_logger().warn(
                    f"受限{fallback_mode}离地爬升率偏大, 先刹车等待交权: "
                    f"高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s, brake_pwm={brake_pwm}"
                )
            emergency_high_rate = (
                climb_rate >= self.takeoff_fallback_emergency_climb_rate
                and now - self._takeoff_fallback_high_rate_since
                >= self.takeoff_fallback_emergency_climb_time
            )
            if emergency_high_rate:
                self.get_logger().error(
                    f"受限{fallback_mode}离地极端爬升率持续超限, 转入降落: "
                    f"高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s, "
                    f"持续={now - self._takeoff_fallback_high_rate_since:.2f}s"
                )
                self.transition(State.LANDING)
            return
        self._takeoff_fallback_high_rate_since = 0.0

        if self._takeoff_fallback_ramp_pwm >= self.takeoff_fallback_ramp_max_pwm:
            if fallback_elapsed > max(12.0, self.takeoff_fallback_timeout):
                self.get_logger().error(
                    f"受限{fallback_mode}离地无足够高度响应! "
                    f"高度={nav_alt:.3f}m, 爬升率={climb_rate:.2f}m/s"
                )
                self.transition(State.ABORT)
                return
        elif fallback_elapsed > max(12.0, self.takeoff_fallback_timeout):
            self.get_logger().error(f"受限{fallback_mode}离地超时! 高度={nav_alt:.3f}m")
            self.transition(State.ABORT)
            return

    def _takeoff_rc_fallback_handoff_reasons(self, nav_alt, climb_rate):
        """Return reasons to keep braking before handing RC liftoff to GUIDED."""
        reasons = []
        now = time.time()
        if self.attitude_rpy is None or now - self._last_attitude_time > 1.0:
            reasons.append("姿态超时")
        else:
            roll, pitch, _ = self.attitude_rpy
            max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            if max_rp > self.takeoff_guided_handoff_max_rp_deg:
                reasons.append(f"姿态未平{max_rp:.1f}deg")

        if self._navigation_speed_xy() > self.hover_speed_xy_max:
            reasons.append(f"水平速度{self._navigation_speed_xy():.2f}m/s")
        if abs(climb_rate) > self.takeoff_fallback_handoff_vertical_rate_max:
            reasons.append(f"垂向速度{climb_rate:.2f}m/s")

        if (
            self.enable_local_position_consistency
            and self.external_nav_position is not None
            and self.local_position is not None
            and now - self._last_local_pos_time <= self.local_position_stale_timeout
        ):
            sx, sy, sz = self.external_nav_position
            lx, ly, lz = self.local_position
            local_err = math.hypot(lx - sx, ly - sy)
            local_z_err = abs(lz - sz)
            if local_err > self.takeoff_guided_handoff_max_local_xy_error:
                reasons.append(f"FCU local/ExternalNav误差{local_err:.2f}m")
            if (
                self.enable_local_position_z_consistency
                and local_z_err > self.takeoff_guided_handoff_max_local_z_error
            ):
                reasons.append(f"FCU local/ExternalNav高度误差{local_z_err:.2f}m")
        elif self.enable_local_position_consistency:
            reasons.append("FCU local位置超时")

        return reasons

    def _update_hover_hold_target(self):
        """Recover to the takeoff centerline while climbing gently."""
        now = time.time()
        nav_alt = self._nav_altitude()
        desired_alt = max(self.hover_target_altitude, self.takeoff_success_altitude)
        if self._guided_hold_target is None:
            hold_x, hold_y = self._current_guided_xy()
            hold_alt = min(desired_alt, max(nav_alt, self.takeoff_success_altitude))
        else:
            hold_x, hold_y, hold_alt, _ = self._guided_hold_target

        dt = max(0.0, min(now - self._last_hover_alt_update, 0.5))
        self._last_hover_alt_update = now

        # After fallback liftoff, first capture the current position, then
        # slowly pull the hover target back to the planned takeoff centerline.
        if self.takeoff_hold_fixed_origin:
            xy_step = max(0.005, self.hover_hold_speed_xy_max) * dt
            err_home_x = -hold_x
            err_home_y = -hold_y
            err_home = math.hypot(err_home_x, err_home_y)
            if err_home <= xy_step or err_home <= 1.0e-4:
                hold_x = 0.0
                hold_y = 0.0
            else:
                hold_x += err_home_x / err_home * xy_step
                hold_y += err_home_y / err_home * xy_step

        climb_rate_limit = max(0.02, self.hover_climb_rate)
        if (
            (self._takeoff_use_rc_fallback or self._takeoff_use_guided_attitude_fallback)
            and self._guided_handoff_done
            and self._guided_handoff_complete_time > 0.0
        ):
            handoff_age = now - self._guided_handoff_complete_time
            overshoot = nav_alt - hold_alt
            still_climbing_above_hold = self._navigation_vertical_rate() > 0.10 and overshoot > 0.15
            if handoff_age < 1.5 and still_climbing_above_hold:
                climb_rate_limit = 0.0
            else:
                climb_rate_limit = min(max(climb_rate_limit, 0.06), 0.08)

        climb_step = climb_rate_limit * dt
        if hold_alt < desired_alt:
            hold_alt = min(desired_alt, hold_alt + climb_step)
        else:
            hold_alt = desired_alt

        self._guided_hold_target = (hold_x, hold_y, hold_alt, "HOVER_STABLE")
        return desired_alt

    def _hover_stability_reasons(self, desired_alt):
        reasons = []
        now = time.time()
        nav_alt = self._nav_altitude()

        if self._guided_hold_target is None:
            reasons.append("无GUIDED悬停目标")
            return reasons

        hold_x, hold_y, hold_alt, _ = self._guided_hold_target
        cur_x, cur_y = self._current_guided_xy()
        xy_err = math.hypot(cur_x - hold_x, cur_y - hold_y)
        home_xy_err = math.hypot(cur_x, cur_y)
        alt_err = abs(nav_alt - desired_alt)
        hold_target_err = abs(hold_alt - desired_alt)
        vertical_rate = abs(self._navigation_vertical_rate())

        if hold_target_err > self.hover_alt_tolerance:
            reasons.append(f"悬停目标未升至任务高度{hold_alt:.2f}/{desired_alt:.2f}m")
        if alt_err > self.hover_alt_tolerance:
            reasons.append(f"高度误差{alt_err:.2f}m")
        if xy_err > self.hover_xy_tolerance:
            reasons.append(f"XY偏移{xy_err:.2f}m")
        if self.takeoff_hold_fixed_origin and home_xy_err > self.hover_xy_tolerance:
            reasons.append(f"起飞中线偏移{home_xy_err:.2f}m")
        if self._navigation_speed_xy() > self.hover_speed_xy_max:
            reasons.append(f"水平速度{self._navigation_speed_xy():.2f}m/s")
        if vertical_rate > self.hover_vertical_rate_max:
            reasons.append(f"垂向速度{vertical_rate:.2f}m/s")
        if self.attitude_rpy is None or now - self._last_attitude_time > 1.0:
            reasons.append("姿态超时")
        else:
            roll, pitch, _ = self.attitude_rpy
            max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            if max_rp > self.hover_max_rp_deg:
                reasons.append(f"姿态未平{max_rp:.1f}deg")

        if (
            self.enable_local_position_consistency
            and self.external_nav_position is not None
            and self.local_position is not None
            and now - self._last_local_pos_time <= self.local_position_stale_timeout
        ):
            sx, sy, _ = self.external_nav_position
            lx, ly, _ = self.local_position
            local_err = math.hypot(lx - sx, ly - sy)
            if local_err > self.hover_local_position_max_xy_error:
                reasons.append(f"FCU local/ExternalNav误差{local_err:.2f}m")

        if (
            self.enable_sim_truth_monitor
            and self.sim_truth_position is not None
            and self.external_nav_position is not None
            and now - self._last_sim_truth_time <= self.sim_truth_stale_timeout
        ):
            tx, ty, _ = self.sim_truth_position
            sx, sy, _ = self.external_nav_position
            truth_err = math.hypot(tx - sx, ty - sy)
            if truth_err > self.sim_truth_max_external_nav_xy_error:
                reasons.append(f"Gazebo真值/ExternalNav误差{truth_err:.2f}m")

        return reasons

    def _maybe_enter_waypoint_nav_after_hover(self, desired_alt):
        reasons = self._hover_stability_reasons(desired_alt)
        now = time.time()
        if reasons:
            self._hover_stable_since = None
            if now - self._last_hover_wait_log > 2.0:
                self._last_hover_wait_log = now
                self.get_logger().info(
                    f"HOVER_STABLE等待: {'; '.join(reasons)} "
                    f"alt={self._nav_altitude():.2f}/{desired_alt:.2f} "
                    f"mode={self.current_mode}"
                )
            if self._elapsed() > self.hover_stable_timeout:
                self.get_logger().error(
                    f"HOVER_STABLE超时仍不稳定: {'; '.join(reasons)}"
                )
                self.transition(State.LANDING)
            return

        if self._hover_stable_since is None:
            self._hover_stable_since = now
            self.get_logger().info("HOVER_STABLE条件满足, 开始连续稳定计时")
            return

        stable_for = now - self._hover_stable_since
        if stable_for < self.hover_stable_time:
            return

        self._wpt_index = 1
        self._wpt_deadline = time.time() + self.waypoint_timeout
        self._guided_hold_target = self._wpt_waypoints[self._wpt_index]
        self._waypoint_arrival_stable_since = None
        self.get_logger().info(
            f"GUIDED悬停稳定{stable_for:.1f}s, 开始航点导航序列"
        )
        self.transition(State.WAYPOINT_NAV)

    def _takeoff_guided_handoff_reasons(self):
        """受限离地后, 进入GUIDED位置控制前的稳定性门槛。"""
        reasons = []
        now = time.time()

        if self.attitude_rpy is None or now - self._last_attitude_time > 1.0:
            reasons.append("飞控姿态超时")
        else:
            roll, pitch, _ = self.attitude_rpy
            max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            if max_rp > self.takeoff_guided_handoff_max_rp_deg:
                reasons.append(f"姿态未平{max_rp:.1f}deg")

        nav_alt = self._nav_altitude()
        required_alt = self.takeoff_guided_handoff_min_altitude
        if self._guided_hold_target is not None:
            _, _, hold_alt, _ = self._guided_hold_target
            required_alt = max(
                required_alt,
                min(hold_alt, self.target_altitude),
                self.takeoff_fallback_liftoff_altitude,
            )

        if nav_alt < required_alt:
            reasons.append(
                f"交权高度不足{nav_alt:.2f}/{required_alt:.2f}m"
            )

        if self._navigation_speed_xy() > self.hover_speed_xy_max:
            reasons.append(f"水平速度{self._navigation_speed_xy():.2f}m/s")
        if abs(self._navigation_vertical_rate()) > self.hover_vertical_rate_max:
            reasons.append(f"垂向速度{self._navigation_vertical_rate():.2f}m/s")

        if (
            self.enable_local_position_consistency
            and self.external_nav_position is not None
            and self.local_position is not None
            and now - self._last_local_pos_time <= self.local_position_stale_timeout
        ):
            sx, sy, sz = self.external_nav_position
            lx, ly, lz = self.local_position
            local_err = math.hypot(lx - sx, ly - sy)
            local_z_err = abs(lz - sz)
            if local_err > self.takeoff_guided_handoff_max_local_xy_error:
                reasons.append(f"FCU local/ExternalNav误差{local_err:.2f}m")
            if (
                self.enable_local_position_z_consistency
                and local_z_err > self.takeoff_guided_handoff_max_local_z_error
            ):
                reasons.append(f"FCU local/ExternalNav高度误差{local_z_err:.2f}m")
        elif self.enable_local_position_consistency:
            reasons.append("FCU local位置超时")

        return reasons

    def _handle_guided_handoff_after_fallback(self):
        """RC/姿态受限离地后, 先用当前位置GUIDED目标交权, 再进入爬升目标。"""
        now = time.time()
        nav_alt = self._nav_altitude()
        if self._guided_hold_target is None:
            hold_x, hold_y = self._current_guided_xy()
            hold_alt = min(
                self.target_altitude,
                max(
                    nav_alt,
                    self.takeoff_success_altitude,
                    self.takeoff_guided_handoff_min_altitude,
                    self.takeoff_fallback_liftoff_altitude,
                ),
            )
            self._guided_hold_target = (hold_x, hold_y, hold_alt, "TAKEOFF_HANDOFF")
        hold_x, hold_y, hold_alt, hold_label = self._guided_hold_target

        if self.current_mode != "GUIDED":
            self._last_guided_target_time = 0.0
            self._publish_guided_position(
                hold_x, hold_y, hold_alt, f"{hold_label}_PRESET"
            )
            self._publish_rc(throttle=self._takeoff_settle_throttle())
            self._request_guided_mode()
            if self._elapsed() > self.guided_mode_timeout:
                self.get_logger().error(
                    f"HOVER_STABLE: GUIDED交权切换超时(当前={self.current_mode})"
                )
                self.transition(State.ABORT)
            return True

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True
            self.get_logger().info("GUIDED位置交权: 已释放RC override, 由飞控位置控制接管")

        if self._guided_handoff_level_since <= 0.0:
            self._guided_handoff_level_since = now
            self._last_guided_target_time = 0.0
            self.get_logger().info(
                "GUIDED位置交权开始: 发布当前位置低空保持目标, 等待FCU local与ExternalNav收敛"
            )

        self._publish_guided_handoff_hold(hold_x, hold_y, hold_alt, hold_label)

        handoff_elapsed = now - self._guided_handoff_level_since
        reasons = self._takeoff_guided_handoff_reasons()
        min_time = max(0.5, self.takeoff_guided_handoff_time)
        if handoff_elapsed < min_time:
            reasons.append(f"位置交权{handoff_elapsed:.1f}/{min_time:.1f}s")

        if reasons:
            if now - self._last_guided_handoff_log >= 1.0:
                self._last_guided_handoff_log = now
                self.get_logger().info(
                    f"GUIDED位置交权等待: {'; '.join(reasons)} "
                    f"alt={nav_alt:.2f}m mode={self.current_mode}"
                )
            if self._elapsed() > self.hover_stable_timeout:
                self.get_logger().error(
                    f"GUIDED位置交权超时: {'; '.join(reasons)}"
                )
                self.transition(State.LANDING)
            return True

        if self.skip_hover_stable_after_takeoff:
            self._start_waypoint_nav_after_takeoff("GUIDED位置交权完成")
            return False

        self._complete_guided_position_handoff(nav_alt, "TAKEOFF_HANDOFF")
        return False

    # ===== HOVER_STABLE =====
    def _handle_hover_stable(self):
        if self.localization_mode:
            if self._takeoff_use_rc_fallback and time.time() < self._takeoff_settle_until:
                if self.current_mode != self.takeoff_fallback_mode:
                    self._request_mode_throttled(self.takeoff_fallback_mode)
                self._publish_rc(throttle=self._takeoff_settle_throttle())
                return
            if self.current_mode != "GUIDED":
                if (
                    (self._takeoff_use_rc_fallback or self._takeoff_use_guided_attitude_fallback)
                    and not self._guided_handoff_done
                ):
                    self._publish_rc(throttle=self._takeoff_settle_throttle())
                self._request_guided_mode()
                if self._elapsed() > self.guided_mode_timeout:
                    self.get_logger().error(
                        f"HOVER_STABLE: GUIDED切换超时(当前={self.current_mode})"
                    )
                    self.transition(State.ABORT)
                return
            if (
                (self._takeoff_use_rc_fallback or self._takeoff_use_guided_attitude_fallback)
                and not self._guided_handoff_done
            ):
                if self._handle_guided_handoff_after_fallback():
                    return
            if not self._rc_released_for_guided:
                self._release_rc_override()
                self._rc_released_for_guided = True
            desired_alt = self._update_hover_hold_target()
            self._publish_guided_hold()
            self._maybe_enter_waypoint_nav_after_hover(desired_alt)
        else:
            self._publish_hover()
            if self._elapsed() > self.hover_stable_time:
                self._wpt_index = 1
                self._wpt_deadline = time.time() + self.waypoint_timeout
                self.transition(State.WAYPOINT_NAV)

    # ===== WAYPOINT_NAV (GUIDED本地目标航点导航) =====
    def _handle_waypoint_nav(self):
        """发布 GUIDED local NED 位置/速度目标, 使用ExternalNav/飞控反馈判定到达。
        遍历 COMPETITION_WAYPOINTS_NED, 到达半径内进入下一航点。
        """
        # 首次进入时发布完整航点路径
        if self._elapsed() < 0.2:
            self._publish_waypoints()

        if self.current_mode != "GUIDED":
            self._request_guided_mode()
            if time.time() > self._guided_mode_deadline:
                self.get_logger().error(
                    f"WAYPOINT_NAV: GUIDED切换超时(当前={self.current_mode})"
                )
                self.transition(State.ABORT)
            return

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True

        nav_position = self._navigation_position()
        if nav_position is None:
            self._publish_guided_hold()
            if self._elapsed() > 10.0:
                self.get_logger().error(
                    f"WAYPOINT_NAV: {self.localization_backend}位置超时, 中止任务"
                )
                self.mode_pub.publish(String(data="ALT_HOLD"))
                self._ekf_abort_triggered = True
                self.transition(State.ABORT)
            return

        # 所有航点完成?
        if self._wpt_index >= len(self._wpt_waypoints):
            self.get_logger().info("所有航点完成! 悬停等待降落")
            self._publish_guided_hold()
            self.transition(State.LANDING)
            return

        wpt = self._wpt_waypoints[self._wpt_index]
        tgt_x, tgt_y, tgt_z, label = wpt
        cur_x, cur_y, cur_z = nav_position

        # 计算位置误差 (NED)
        err_x = tgt_x - cur_x  # 北向误差
        err_y = tgt_y - cur_y  # 东向误差
        err_z = tgt_z - self._nav_altitude()  # 高度误差

        # B2修复: ExternalNav输出NaN检测 (滤波器未收敛/协方差爆炸)
        if not (math.isfinite(err_x) and math.isfinite(err_y) and math.isfinite(err_z)):
            self.get_logger().error(
                f"GUIDED导航输入异常! err=({err_x},{err_y},{err_z}) external_nav=({cur_x},{cur_y},{cur_z})"
            )
            self._publish_guided_hold()
            return

        dist_xy = math.sqrt(err_x**2 + err_y**2)
        use_velocity_nav = (
            self.guided_nav_control_mode == "velocity" or self.guided_3d_enabled
        )
        cmd_x, cmd_y, cross_track, path_along, path_len, cmd_label, seg_ux, seg_uy = (
            self._compute_segment_follow_target(cur_x, cur_y, tgt_x, tgt_y, label)
        )
        cmd_err_x = cmd_x - cur_x
        cmd_err_y = cmd_y - cur_y
        cmd_dist_xy = math.hypot(cmd_err_x, cmd_err_y)
        self._guided_hold_target = (cmd_x, cmd_y, tgt_z, cmd_label)
        if use_velocity_nav:
            attitude_brake = False
            now = time.time()
            if self.attitude_rpy is not None and now - self._last_attitude_time <= 1.0:
                roll, pitch, _ = self.attitude_rpy
                max_rp = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
                attitude_brake = max_rp > self.guided_nav_attitude_brake_rp_deg
                if attitude_brake and now - getattr(self, "_last_nav_attitude_brake_log", 0.0) > 1.0:
                    self._last_nav_attitude_brake_log = now
                    self.get_logger().warn(
                        f"WAYPOINT_NAV: 姿态{max_rp:.1f}deg偏大, 暂停XY推进等待回稳"
                    )
            if self._guided_velocity_brake_active:
                velocity_brake = self._navigation_speed_xy() > self.guided_velocity_resume_speed_xy
            else:
                velocity_brake = self._navigation_speed_xy() > self.guided_velocity_brake_speed_xy
            self._guided_velocity_brake_active = velocity_brake
            if velocity_brake and now - self._last_velocity_brake_log > 1.0:
                self._last_velocity_brake_log = now
                self.get_logger().warn(
                    f"WAYPOINT_NAV: XY速度{self._navigation_speed_xy():.2f}m/s偏大, "
                    "速度目标刹车等待回稳"
                )
            if path_len > 1.0e-3:
                remaining = max(0.0, path_len - path_along)
                along_speed = min(self.guided_nav_speed_max, self.guided_nav_k_xy * remaining)
                # path_along is deliberately clamped to the segment for
                # monotonic progress. Once the aircraft passes the endpoint,
                # however, "remaining" becomes zero and cannot command a
                # return. Recover the signed endpoint error explicitly so an
                # overshoot is actively driven back instead of coasting.
                endpoint_along_error = err_x * seg_ux + err_y * seg_uy
                if endpoint_along_error < 0.0:
                    along_speed = max(
                        -self.guided_nav_speed_max,
                        self.guided_nav_k_xy * endpoint_along_error,
                    )
                arrival_slow_radius = max(0.90, WAYPOINT_REACHED_RADIUS * 3.0)
                if dist_xy < arrival_slow_radius:
                    slow_scale = max(0.15, min(1.0, dist_xy / arrival_slow_radius))
                    along_speed *= slow_scale
                    along_speed = min(along_speed, self.guided_nav_k_xy * dist_xy)
                if abs(cross_track) > self.guided_nav_cross_track_stop:
                    along_speed = 0.0
                elif abs(cross_track) > self.guided_path_cross_track_warn:
                    scale = 1.0 - (
                        (abs(cross_track) - self.guided_path_cross_track_warn)
                        / max(
                            self.guided_nav_cross_track_stop - self.guided_path_cross_track_warn,
                            1.0e-3,
                        )
                    )
                    along_speed *= max(0.0, min(1.0, scale))
                if (
                    self.guided_nav_min_along_speed > 0.0
                    and remaining > WAYPOINT_REACHED_RADIUS
                    and abs(cross_track) <= self.guided_path_cross_track_warn
                    and dist_xy >= arrival_slow_radius
                ):
                    along_speed = max(along_speed, self.guided_nav_min_along_speed)
                    along_speed = min(along_speed, self.guided_nav_speed_max)

                cur_vx, cur_vy, _ = self._navigation_velocity()
                cross_rate = cur_vx * (-seg_uy) + cur_vy * seg_ux
                lateral_speed = (
                    -self.guided_nav_cross_track_k * cross_track
                    - self.guided_nav_cross_track_d * cross_rate
                )
                lateral_speed = max(
                    -self.guided_nav_cross_track_speed_max,
                    min(self.guided_nav_cross_track_speed_max, lateral_speed),
                )
                if attitude_brake:
                    along_speed = 0.0
                    lateral_speed = 0.0
                if velocity_brake and self.guided_velocity_brake_damping > 0.0:
                    vx = -cur_vx * self.guided_velocity_brake_damping
                    vy = -cur_vy * self.guided_velocity_brake_damping
                    brake_speed = math.hypot(vx, vy)
                    if (
                        self.guided_velocity_brake_cmd_max > 0.0
                        and brake_speed > self.guided_velocity_brake_cmd_max
                    ):
                        scale = self.guided_velocity_brake_cmd_max / brake_speed
                        vx *= scale
                        vy *= scale
                    vz = self._guided_ned_vz_for_altitude(tgt_z)
                    self._publish_guided_velocity(
                        vx,
                        vy,
                        vz,
                        0.0,
                        f"{label}_BRAKE",
                        force=True,
                        bypass_slew=True,
                    )
                    return
                else:
                    if velocity_brake:
                        along_speed = 0.0
                        lateral_speed = 0.0
                    vx = seg_ux * along_speed + (-seg_uy) * lateral_speed
                    vy = seg_uy * along_speed + seg_ux * lateral_speed

                speed_xy = math.hypot(vx, vy)
                speed_cap = max(self.guided_nav_speed_max, self.guided_nav_cross_track_speed_max)
                if speed_xy > speed_cap:
                    scale = speed_cap / speed_xy
                    vx *= scale
                    vy *= scale
            else:
                speed = min(self.guided_nav_speed_max, self.guided_nav_k_xy * cmd_dist_xy)
                if cmd_dist_xy > 1.0e-3:
                    vx = cmd_err_x / cmd_dist_xy * speed
                    vy = cmd_err_y / cmd_dist_xy * speed
                else:
                    vx = 0.0
                    vy = 0.0
            if self.guided_3d_enabled and not attitude_brake and not velocity_brake:
                # Run the same bounded controller in XYZ.  The legacy
                # segment terms remain useful for diagnostics and cross-track
                # logging, while this single command keeps vertical and
                # horizontal damping consistent for optical-flow odometry.
                start_wpt = self._wpt_waypoints[max(0, self._wpt_index - 1)]
                current_up = (cur_x, cur_y, -cur_z)
                target_up = (tgt_x, tgt_y, tgt_z)
                start_up = (start_wpt[0], start_wpt[1], start_wpt[2])
                lookahead = lookahead_target_3d(
                    current_up, start_up, target_up, self.guided_path_lookahead
                )
                vel_up = (
                    self._navigation_velocity()[0],
                    self._navigation_velocity()[1],
                    -self._navigation_velocity()[2],
                )
                cmd_up = self.guided_controller_3d.velocity_command(
                    current_up, vel_up, lookahead.point
                )
                vx, vy, vz = cmd_up[0], cmd_up[1], -cmd_up[2]
            else:
                vz = self._guided_ned_vz_for_altitude(tgt_z)
            self._publish_guided_velocity(vx, vy, vz, 0.0, cmd_label)
        else:
            position_brake = self._position_nav_brake_needed()
            use_direct_position = False
            if position_brake:
                if self._guided_position_brake_target is None:
                    self._guided_position_brake_target = (cur_x, cur_y)
                cmd_x, cmd_y = self._guided_position_brake_target
                cmd_label = f"{label}_BRAKE"
            elif self._guided_position_brake_target is not None:
                self._guided_position_brake_target = None

            if (
                not position_brake
                and
                self.guided_position_strategy == "direct"
                and abs(cross_track) <= self.guided_path_cross_track_warn
            ):
                use_direct_position = True
                cmd_x, cmd_y, cmd_label = tgt_x, tgt_y, f"{label}_DIRECT"
                cmd_err_x = cmd_x - cur_x
                cmd_err_y = cmd_y - cur_y
                cmd_dist_xy = math.hypot(cmd_err_x, cmd_err_y)
            elif self.guided_position_strategy == "direct":
                cmd_label = f"{cmd_label}_RECOVER"
            # A direct competition waypoint is intentionally fixed. Limiting
            # it relative to the *current* vehicle position creates a moving
            # target which follows an overshooting aircraft and delays
            # braking. ArduPilot already applies WPNAV speed, acceleration and
            # angle limits to the fixed target. Only segment recovery targets
            # need the additional local step limiter.
            if not use_direct_position:
                cmd_x, cmd_y, cmd_label = self._limit_guided_position_step(
                    cur_x, cur_y, cmd_x, cmd_y, cmd_label
                )
            self._guided_hold_target = (cmd_x, cmd_y, tgt_z, cmd_label)
            self._publish_guided_position(cmd_x, cmd_y, tgt_z, cmd_label)

        # 到达判定
        if dist_xy < WAYPOINT_REACHED_RADIUS and abs(err_z) < 0.2:
            if self._navigation_speed_xy() > self.waypoint_reached_speed_xy_max:
                self._waypoint_arrival_stable_since = None
                now = time.time()
                self._guided_hold_target = (tgt_x, tgt_y, tgt_z, f"{label}_ARRIVAL_BRAKE")
                if use_velocity_nav:
                    cur_vx, cur_vy, _ = self._navigation_velocity()
                    vx = -self.guided_velocity_brake_damping * cur_vx
                    vy = -self.guided_velocity_brake_damping * cur_vy
                    brake_speed = math.hypot(vx, vy)
                    if (
                        self.guided_velocity_brake_cmd_max > 0.0
                        and brake_speed > self.guided_velocity_brake_cmd_max
                    ):
                        scale = self.guided_velocity_brake_cmd_max / brake_speed
                        vx *= scale
                        vy *= scale
                    vz = self._guided_ned_vz_for_altitude(tgt_z)
                    self._publish_guided_velocity(
                        vx,
                        vy,
                        vz,
                        0.0,
                        f"{label}_ARRIVAL_BRAKE",
                        force=True,
                        bypass_slew=True,
                    )
                else:
                    self._publish_guided_position(
                        tgt_x,
                        tgt_y,
                        tgt_z,
                        f"{label}_ARRIVAL_BRAKE",
                    )
                if now - self._last_waypoint_arrival_brake_log > 1.0:
                    self._last_waypoint_arrival_brake_log = now
                    self.get_logger().info(
                        f"航点[{self._wpt_index}] {label} 已进半径但速度"
                        f"{self._navigation_speed_xy():.2f}m/s偏大, 先刹稳再切阶段"
                    )
                return
            now = time.time()
            if self._waypoint_arrival_stable_since is None:
                self._waypoint_arrival_stable_since = now
                self.get_logger().info(
                    f"航点[{self._wpt_index}] {label} 已进入到达窗口: "
                    f"dist={dist_xy:.2f}m vxy={self._navigation_speed_xy():.2f}m/s, "
                    "等待连续稳定"
                )
                return
            stable_for = now - self._waypoint_arrival_stable_since
            if stable_for < self.waypoint_arrival_settle_time:
                return
            self.get_logger().info(
                f"航点到达: [{self._wpt_index}] {label} "
                f"({cur_x:.2f},{cur_y:.2f}) → ({tgt_x:.2f},{tgt_y:.2f}) "
                f"距离={dist_xy:.2f}m 稳定={stable_for:.1f}s"
            )
            self._guided_hold_target = wpt
            self._wpt_index += 1
            self._waypoint_arrival_stable_since = None
            self._wpt_deadline = time.time() + self.waypoint_timeout
            self._last_waypoint_timeout_log = 0.0
            self._guided_position_brake_target = None
            if label == "RETURN_TAKEOFF_ZONE":
                self.return_frame_done = True
                self.return_pillars_done = True
                self.get_logger().info("规则进度: 原路返程穿框完成, UAV已回到起飞区中线")
            # 特殊航点:
            if label == "SCAN_ZONE":         # 框后 → 地面码识别
                self.get_logger().info("到达地面码识别区, 切到码识别")
                self.transition(State.CODE_SEARCH)
                return
            if label == "STRIKE_ZONE":       # UGV最终停车/扎球点 → 等待UGV
                self.return_pillars_done = True
                self.uav_back_in_strike_zone = True
                self.get_logger().info("规则进度: UAV已到UGV最终停车/扎球点, 等待UGV和气球就位")
                self._publish_ugv_start_once()
                self.transition(State.WAIT_UGV_READY)
                return
            if label == "BALLOON_ALT":       # WP8 → 气球搜索
                self.get_logger().info("到达气球高度, 切到视觉搜索")
                self.transition(State.BALLOON_SEARCH)
                return
            if label == "LAND":              # WP9 → 降落
                self.transition(State.LANDING)
                return
            return
        else:
            self._waypoint_arrival_stable_since = None

        # 超时处理: 比赛关键航点不能直接跳过，否则会在错误航段上继续飞。
        if time.time() > self._wpt_deadline:
            if self.allow_waypoint_timeout_skip:
                self.get_logger().warn(f"航点 [{self._wpt_index}] {label} 超时, 参数允许跳过")
                skipped_index = self._wpt_index
                self._wpt_index += 1
                self._waypoint_arrival_stable_since = None
                self._wpt_deadline = time.time() + self.waypoint_timeout
                self._guided_segment_progress.pop(skipped_index, None)
                self._guided_segment_progress_time.pop(skipped_index, None)
                return
            now = time.time()
            if now - self._last_waypoint_timeout_log > 5.0:
                self._last_waypoint_timeout_log = now
                self.get_logger().warn(
                    f"航点 [{self._wpt_index}] {label} 超时, 规则模式不跳过; "
                    f"继续低速重发目标 dist={dist_xy:.2f}m cross={cross_track:.2f}m"
                )
            self._wpt_deadline = time.time() + self.waypoint_timeout
            return

        # 每3秒导航状态 + 更新航点标记
        now = time.time()
        if now - getattr(self, '_wpt_last_log', 0.0) > 3.0:
            self._wpt_last_log = now
            self._publish_waypoints()
            self.get_logger().info(
                f"[NAV] WP{self._wpt_index}={label} "
                f"pos=({cur_x:.2f},{cur_y:.2f})→({tgt_x:.2f},{tgt_y:.2f}) "
                f"err=({err_x:.2f},{err_y:.2f}) dist={dist_xy:.2f}m "
                f"path={path_along:.2f}/{path_len:.2f}m cross={cross_track:.2f}m "
                f"GUIDED {self.guided_nav_control_mode} cmd_ned=({cmd_x:.2f},{cmd_y:.2f},{-tgt_z:.2f})"
            )

    # ===== CODE_SEARCH =====
    def _handle_code_search(self):
        self._publish_guided_hold()
        if self.code_detected and self.code_data:
            if not self.code_task_done:
                self.code_task_done = True
                self._code_success_since = time.time()
                self.get_logger().info(f"地面码识别成功: {self.code_data}")
            hold_elapsed = time.time() - (self._code_success_since or time.time())
            if (
                hold_elapsed < self.code_success_hold_time
                or self._navigation_speed_xy() > self.code_success_settle_speed
            ):
                now = time.time()
                if now - self._last_code_settle_log > 1.0:
                    self._last_code_settle_log = now
                    self.get_logger().info(
                        f"码识别后稳定等待: hold={hold_elapsed:.1f}/"
                        f"{self.code_success_hold_time:.1f}s "
                        f"vxy={self._navigation_speed_xy():.2f}/"
                        f"{self.code_success_settle_speed:.2f}m/s"
                    )
                return
            if not self._resume_waypoints_through_label("RETURN_TAKEOFF_ZONE"):
                return
            self.transition(State.WAYPOINT_NAV)
        elif self._elapsed() > 20.0:
            if not self.require_code_success:
                self.get_logger().warn("码识别超时, 当前参数允许放弃码识别后继续返程")
                if not self._resume_waypoints_through_label("RETURN_TAKEOFF_ZONE"):
                    return
                self.transition(State.WAYPOINT_NAV)
                return
            self.get_logger().error(
                "码识别超时: 规则要求先成功识别地面码, 不允许跳过后继续扎气球"
            )
            if self.code_failure_action == "ABORT":
                self.transition(State.ABORT)
            elif self.code_failure_action == "RETURN_HOME":
                if not self._resume_waypoints_through_label("RETURN_TAKEOFF_ZONE"):
                    return
                self.transition(State.WAYPOINT_NAV)
            else:
                self.transition(State.LANDING)

    # ===== WAIT_UGV =====
    def _handle_wait_ugv(self):
        self._publish_guided_hold()
        self._publish_strike_permission(False)

        if self.require_code_success and not self.code_task_done:
            self.get_logger().error("未完成码识别却进入UGV等待, 按规则停止扎气球流程")
            self.transition(State.LANDING)
            return
        if not self.uav_back_in_strike_zone:
            self.get_logger().error("UAV未到UGV最终停车/扎球点却等待扎气球, 按规则停止")
            self.transition(State.LANDING)
            return

        self._publish_ugv_start_once()

        if self._rules_allow_strike():
            self.get_logger().info("UGV和气球已停到最终扎球点, 规则门禁通过, 继续气球航点")
            self._publish_strike_permission(True)
            if not self._resume_waypoints_through_label("BALLOON_ALT"):
                return
            self.transition(State.WAYPOINT_NAV)
            return

        if self._elapsed() > self.ugv_wait_timeout:
            if self.allow_strike_without_ugv:
                self.get_logger().warn(
                    f"等待UGV超时({self.ugv_wait_timeout:.0f}s), 当前参数允许调试越权进入气球阶段"
                )
                self._publish_strike_permission(True)
                if not self._resume_waypoints_through_label("BALLOON_ALT"):
                    return
                self.transition(State.WAYPOINT_NAV)
                return
            if not self._ugv_timeout_logged:
                self._ugv_timeout_logged = True
                self.get_logger().warn(
                    f"等待UGV超时({self.ugv_wait_timeout:.0f}s), 规则模式下继续悬停等待, "
                    "不发布strike_permission"
                )

    # ===== BALLOON_SEARCH =====
    def _handle_balloon_search(self):
        if not self._rules_allow_strike():
            self._deny_balloon_stage(
                "需要码识别、返程穿框/避障、UAV到最终扎球点、UGV和气球停到最终扎球点全部完成"
            )
            return
        self._publish_strike_permission(True)
        if self.simulate_balloon_puncture_on_detection:
            self._publish_guided_hold()
            if self.red_ball_found:
                self.get_logger().info("仿真红气球已检测到, 直接确认穿刺命中")
                self.transition(State.PUNCTURE_CONFIRM)
            elif self._elapsed() > 15.0:
                self.get_logger().warn("仿真搜索气球超时, 进入降落")
                self.transition(State.LANDING)
            return

        if self.current_mode != "LOITER":
            self._request_loiter_mode()
            self._publish_guided_hold()
            return
        self._rc_released_for_guided = False
        # RC 由 visual_servo_node 接管 — 状态机不发布, 避免冲突
        if self.red_ball_found:
            self.get_logger().info("发现红气球!")
            self.transition(State.BALLOON_ALIGN)
        elif self._elapsed() > 30.0:
            self.get_logger().warn("搜索气球超时!")
            self.transition(State.LANDING)

    # ===== BALLOON_ALIGN =====
    def _handle_balloon_align(self):
        if not self._rules_allow_strike():
            self._deny_balloon_stage("气球对准前规则门禁失效")
            return
        self._publish_strike_permission(True)
        if not self.red_ball_found:
            self.transition(State.BALLOON_SEARCH)
            return
        if self.red_ball_aligned:
            self.get_logger().info("气球已对准! 准备下降")
            self.transition(State.PUNCTURE_DESCEND)
        # visual_servo_node 负责伺服控制
        if self._elapsed() > 20.0:
            self.get_logger().warn("对准超时!")
            self.transition(State.LANDING)

    # ===== PUNCTURE_DESCEND =====
    def _handle_puncture_descend(self):
        if not self._rules_allow_strike():
            self._deny_balloon_stage("穿刺下降前规则门禁失效")
            return
        self._publish_strike_permission(True)
        if not self.red_ball_found:
            self.get_logger().warn("丢失气球!")
            self.transition(State.BALLOON_SEARCH)
            return
        if self.current_alt < 0.4:
            self.get_logger().warn("高度过低!")
            self.transition(State.LANDING)
            return
        # visual_servo_node 在收到 strike_permission 后会自动缓降
        if self._elapsed() > 15.0:
            self.get_logger().info("扎气球完成!")
            self.transition(State.PUNCTURE_CONFIRM)

    # ===== PUNCTURE_CONFIRM =====
    def _handle_puncture_confirm(self):
        self._publish_strike_permission(False)
        self._publish_balloon_punctured(True)
        self._release_rc_override()

        if self.simulate_balloon_puncture_on_detection:
            self.get_logger().info("仿真气球穿刺确认: 已发布balloon_punctured=True")
            self._mission_success = True
            self._mission_end_reason = "已完成穿框、码识别、UGV到位和扎球确认"
            if self.post_puncture_return_home and self.is_armed:
                self.transition(State.RETURN_HOME_AFTER_STRIKE)
            else:
                self.transition(State.LANDING if self.is_armed else State.COMPLETE)
            return

        self.get_logger().info("气球扎破确认, 切回GUIDED返航/降落")
        self._mission_success = True
        self._mission_end_reason = "已完成穿框、码识别、UGV到位和扎球确认"
        if self.post_puncture_return_home and self.is_armed:
            self.transition(State.RETURN_HOME_AFTER_STRIKE)
        elif self._elapsed() > 0.5:
            self.transition(State.LANDING)

    # ===== RETURN_HOME_AFTER_STRIKE =====
    def _handle_return_home_after_strike(self):
        self._publish_strike_permission(False)
        self._publish_balloon_punctured(True)

        if not self.is_armed:
            self.transition(State.COMPLETE)
            return

        if self.current_mode != "GUIDED":
            self._request_guided_mode()
            if time.time() > self._guided_mode_deadline:
                self.get_logger().warn("扎破后返航: GUIDED切换超时, 直接进入降落")
                self.transition(State.LANDING)
            return

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True

        nav_position = self._navigation_position()
        if nav_position is None:
            self._publish_guided_hold()
            if time.time() > self._post_strike_deadline:
                self.get_logger().warn("扎破后返航: 定位丢失超时, 进入降落")
                self.transition(State.LANDING)
            return

        cur_x, cur_y, _ = nav_position
        target_x = 0.0
        target_y = 0.0
        target_alt = max(self.post_puncture_return_altitude, self.target_altitude)
        nav_alt = self._nav_altitude()
        err_xy = math.hypot(target_x - cur_x, target_y - cur_y)
        err_z = target_alt - nav_alt

        if self._post_strike_phase == "CLIMB":
            if self._post_strike_anchor_xy is None:
                self._post_strike_anchor_xy = (cur_x, cur_y)
                self.get_logger().info(
                    f"扎破后收尾: 先在打击区上方原地爬升到{target_alt:.2f}m, "
                    "再低速返航"
                )
            hold_x, hold_y = self._post_strike_anchor_xy
            self._guided_hold_target = (hold_x, hold_y, target_alt, "POST_STRIKE_CLIMB")
            self._publish_guided_position(hold_x, hold_y, target_alt, "POST_STRIKE_CLIMB")

            climb_ready = abs(err_z) <= self.post_puncture_climb_tolerance
            if climb_ready:
                if self._post_strike_climb_stable_since is None:
                    self._post_strike_climb_stable_since = time.time()
                elif (
                    time.time() - self._post_strike_climb_stable_since
                    >= self.post_puncture_climb_settle_time
                ):
                    self._post_strike_phase = "RETURN_Y"
                    self.get_logger().info(
                        f"扎破后收尾: 安全高度已稳定, 先沿安全走廊分段返航 "
                        f"dist={err_xy:.2f}m alt={nav_alt:.2f}m"
                    )
            else:
                self._post_strike_climb_stable_since = None

            if time.time() > self._post_strike_deadline:
                self.get_logger().warn(
                    f"扎破后原地爬升超时: alt={nav_alt:.2f}/{target_alt:.2f}m, 进入降落"
                )
                self.transition(State.LANDING)
            return

        anchor_xy = self._post_strike_anchor_xy or (cur_x, cur_y)
        if self._post_strike_phase not in ("RETURN_Y", "RETURN_X"):
            self._post_strike_phase = "RETURN_Y"
        target_x, target_y = staged_return_target(
            anchor_xy, self._post_strike_phase
        )
        err_xy = math.hypot(target_x - cur_x, target_y - cur_y)
        self._guided_hold_target = (
            target_x,
            target_y,
            target_alt,
            f"POST_STRIKE_{self._post_strike_phase}",
        )
        if self.guided_nav_control_mode == "velocity":
            speed = min(self.guided_nav_speed_max, self.guided_nav_k_xy * err_xy)
            near_home = err_xy <= self.post_puncture_home_brake_radius
            if (
                self.post_puncture_return_speed_min > 0.0
                and not near_home
            ):
                speed = max(speed, self.post_puncture_return_speed_min)
                speed = min(speed, self.guided_nav_speed_max)
            if near_home:
                # The takeoff point is close to the west wall. Enter a wide
                # terminal area before reaching it and cap the *base* return
                # speed as well as adding measured-velocity damping. Without
                # this cap, diagonal return can carry residual cross-track
                # speed past x=0 even though the radial error is still large.
                speed = min(speed, self.post_puncture_home_speed_max)
            if err_xy > 1.0e-3:
                vx = (target_x - cur_x) / err_xy * speed
                vy = (target_y - cur_y) / err_xy * speed
            else:
                vx = 0.0
                vy = 0.0
            if near_home and self.post_puncture_home_brake_damping > 0.0:
                cur_vx, cur_vy, _ = self._navigation_velocity()
                vx -= self.post_puncture_home_brake_damping * cur_vx
                vy -= self.post_puncture_home_brake_damping * cur_vy
                brake_cmd_cap = max(0.08, self.post_puncture_home_speed_max * 2.0)
                brake_cmd_speed = math.hypot(vx, vy)
                if brake_cmd_speed > brake_cmd_cap:
                    scale = brake_cmd_cap / brake_cmd_speed
                    vx *= scale
                    vy *= scale
            vz = self._guided_ned_vz_for_altitude(target_alt)
            self._publish_guided_velocity(
                vx,
                vy,
                vz,
                0.0,
                (
                    f"POST_STRIKE_{self._post_strike_phase}_BRAKE"
                    if near_home
                    else f"POST_STRIKE_{self._post_strike_phase}"
                ),
                force=near_home,
                bypass_slew=near_home,
            )
        else:
            cmd_x, cmd_y = staged_return_step(
                (cur_x, cur_y),
                anchor_xy,
                self._post_strike_phase,
                self.post_puncture_return_step_distance,
            )
            cmd_label = f"POST_STRIKE_{self._post_strike_phase}_STEP"
            self._guided_hold_target = (cmd_x, cmd_y, target_alt, cmd_label)
            self._publish_guided_position(cmd_x, cmd_y, target_alt, cmd_label)

        if (
            self._post_strike_phase == "RETURN_Y"
            and abs(cur_y - target_y) <= self.post_puncture_home_radius
            and self._navigation_speed_xy() <= self.post_puncture_home_speed_max
        ):
            self._post_strike_phase = "RETURN_X"
            target_x, target_y = staged_return_target(
                anchor_xy, self._post_strike_phase
            )
            err_xy = math.hypot(target_x - cur_x, target_y - cur_y)
            self._post_strike_home_stable_since = None
            self.get_logger().info(
                "扎破后返航: 已稳定进入起飞点安全走廊, 开始短距离位置修正"
            )

        home_ready = (
            self._post_strike_phase == "RETURN_X"
            and err_xy <= self.post_puncture_home_radius
            and abs(err_z) < 0.25
            and self._navigation_speed_xy() <= self.post_puncture_home_speed_max
        )
        if home_ready:
            if self._post_strike_home_stable_since is None:
                self._post_strike_home_stable_since = time.time()
                self.get_logger().info(
                    f"扎破后已进入起飞点精确着陆窗口: dist={err_xy:.2f}m "
                    f"vxy={self._navigation_speed_xy():.2f}m/s, 等待稳定"
                )
            elif (
                time.time() - self._post_strike_home_stable_since
                >= self.post_puncture_home_settle_time
            ):
                self.get_logger().info(
                    f"扎破后返航制动完成: dist={err_xy:.2f}m "
                    f"vxy={self._navigation_speed_xy():.2f}m/s, 开始闭环降落"
                )
                self.transition(State.LANDING)
                return
        else:
            self._post_strike_home_stable_since = None

        if time.time() > self._post_strike_deadline:
            self.get_logger().warn(
                f"扎破后返航超时: dist={err_xy:.2f}m alt_err={err_z:.2f}m, 进入降落"
            )
            self.transition(State.LANDING)

    # ===== LANDING =====
    def _handle_landing(self):
        self._publish_strike_permission(False)
        if self.native_autopilot_landing:
            self._handle_native_landing()
            return
        if self.landing_strategy == "GUIDED_DESCEND":
            self._handle_guided_landing()
            return
        if self._rc_landing_active():
            self._handle_rc_landing()
            return

        self.land_pub.publish(Bool(data=True))
        # 2秒后主动解锁，防止飞控着陆后保持armed导致死锁 (C12修复)
        if self._elapsed() > 2.0:
            self.arm_pub.publish(Bool(data=False))
        if (self.current_alt < 0.15 and not self.is_armed) or self._elapsed() > 30.0:
            self.transition(State.COMPLETE if self._mission_success else State.ABORT)

    def _handle_native_landing(self):
        """Let LAND detect contact; never cut motors based on altitude alone."""
        now = time.time()
        nav_alt = self._nav_altitude()
        grounded = (self.fcu_landed and now - self._last_landed_time < 3.
                    and nav_alt <= .15 and abs(self._navigation_vertical_rate()) < .10)
        if self.is_armed:
            if self.current_mode != 'LAND':
                self._request_mode_throttled('LAND')
            if grounded:
                if self._landing_touchdown_since is None:
                    self._landing_touchdown_since = now
                elif now - self._landing_touchdown_since >= 1.:
                    self.arm_pub.publish(Bool(data=False))
            else:
                self._landing_touchdown_since = None
        else:
            if nav_alt > .25:
                self.get_logger().error('原生降落在空中意外上锁，任务中止')
                self.transition(State.ABORT)
                return
            level = (self.attitude_rpy is not None
                     and max(abs(self.attitude_rpy[0]), abs(self.attitude_rpy[1])) < math.radians(15.))
            if grounded and level:
                if self._landing_settled_since is None:
                    self._landing_settled_since = now
                elif now - self._landing_settled_since >= 2.:
                    self.transition(State.COMPLETE if self._mission_success else State.ABORT)
            else:
                self._landing_settled_since = None
        if self._elapsed() > 90.:
            self.get_logger().error('原生LAND接地/稳定确认超时，任务中止')
            self.transition(State.ABORT)

    def _handle_guided_landing(self):
        """Descend in GUIDED while retaining an XY hold over the takeoff point."""
        if not self.is_armed:
            if self._nav_altitude() <= self.landing_ground_altitude + 0.10:
                self.transition(State.COMPLETE if self._mission_success else State.ABORT)
            else:
                self.get_logger().error("GUIDED降落在空中意外解锁, 任务中止")
                self.transition(State.ABORT)
            return

        if self.current_mode != "GUIDED":
            self._request_guided_mode()
            if self._elapsed() > 5.0:
                self.get_logger().error("GUIDED降落模式切换超时, 改用飞控LAND保护")
                self.land_pub.publish(Bool(data=True))
            return

        if not self._rc_released_for_guided:
            self._release_rc_override()
            self._rc_released_for_guided = True

        nav_position = self._navigation_position()
        if nav_position is None:
            self.get_logger().error("GUIDED降落丢失外部定位, 改用飞控LAND保护")
            self.land_pub.publish(Bool(data=True))
            return

        cur_x, cur_y, _ = nav_position
        cur_vx, cur_vy, _ = self._navigation_velocity()
        vx = -self.guided_nav_k_xy * cur_x - self.landing_xy_damping * cur_vx
        vy = -self.guided_nav_k_xy * cur_y - self.landing_xy_damping * cur_vy
        speed_xy = math.hypot(vx, vy)
        if speed_xy > self.landing_xy_speed_max:
            scale = self.landing_xy_speed_max / speed_xy
            vx *= scale
            vy *= scale

        nav_alt = self._nav_altitude()
        on_ground = nav_alt <= self.landing_ground_altitude
        descent_rate = max(0.05, self.landing_guided_descent_rate)
        self._publish_guided_velocity(
            vx,
            vy,
            0.0 if on_ground else descent_rate,
            0.0,
            "GUIDED_LANDING_XY_HOLD",
            force=on_ground,
            bypass_slew=on_ground,
        )

        if on_ground:
            self.arm_pub.publish(Bool(data=False))

        hard_timeout = max(30.0, self.landing_disarm_delay + 12.0)
        if self._elapsed() > hard_timeout:
            if nav_alt <= self.landing_ground_altitude + 0.10:
                self.arm_pub.publish(Bool(data=False))
            else:
                self.get_logger().error(
                    f"GUIDED降落超时且仍在空中: alt={nav_alt:.2f}m, "
                    "转飞控LAND并判定任务中止"
                )
                self.land_pub.publish(Bool(data=True))
                self.transition(State.ABORT)

    def _handle_rc_landing(self):
        mode = "STABILIZE" if self.landing_strategy == "STABILIZE_RC" else "ALT_HOLD"
        if self.current_mode != mode:
            self._request_mode_throttled(mode)

        throttle = self._landing_descent_throttle()
        self._publish_rc(1500, 1500, throttle, 1500)
        if self.landing_guided_descent_rate > 0.0 and self.current_alt > 0.12:
            # Add a bounded NED descent target to the RC landing command so the
            # Gazebo/SITL chain continues descending at very low throttle.
            self._publish_guided_velocity(
                0.0,
                0.0,
                self.landing_guided_descent_rate,
                0.0,
                "LANDING_DESCEND",
            )

        if self.current_alt < 0.15 or self._elapsed() > self.landing_disarm_delay:
            self.arm_pub.publish(Bool(data=False))
        hard_timeout = max(20.0, self.landing_disarm_delay + 5.0)
        if (
            (self.current_alt < 0.15 and not self.is_armed)
            or self._elapsed() > hard_timeout
        ):
            self.transition(State.COMPLETE if self._mission_success else State.ABORT)

    def _handle_safe_stop(self):
        self.strike_perm_pub.publish(Bool(data=False))
        if self.current_mode != self.safe_stop_mode:
            self._request_mode_throttled(self.safe_stop_mode)
        self._publish_rc(1500, 1500, self._safe_stop_throttle(), 1500)
        if (
            self.current_alt < 0.15
            or (
                self.safe_stop_disarm_delay > 0.0
                and self._elapsed() > self.safe_stop_disarm_delay
            )
        ):
            self.arm_pub.publish(Bool(data=False))
        if not self.is_armed and self._elapsed() > self.safe_stop_disarm_delay:
            self.get_logger().error("SAFE_STOP完成: 飞机已上锁, 本轮任务未完成")
            self.transition(State.ABORT)
        elif self._elapsed() > 20.0:
            self.get_logger().error("SAFE_STOP超时仍未上锁, 转入ABORT继续上锁")
            self.transition(State.ABORT)

    # ===== COMPLETE =====
    def _handle_complete(self):
        elapsed = self._elapsed()
        # 第一tick打印完成信息并清理mission计时器
        if self.mission_start_time is not None or self.mission_started:
            total = time.time() - self.mission_start_time if self.mission_start_time else 0
            status = "任务完成" if self._mission_success else "任务结束但未完成"
            reason = self._mission_end_reason or "未记录完成原因"
            self.get_logger().info(
                f"=== {status}! 总耗时={total:.1f}s "
                f"模式={self.current_mode} armed={self.is_armed} alt={self.current_alt:.2f}m "
                f"reason={reason} ==="
            )
            self.mission_started = False
            self.mission_start_time = None

        if elapsed < 1.0:
            if self.is_armed:
                self._publish_rc(1500, 1500, self._safe_stop_throttle(), 1500)
                self.arm_pub.publish(Bool(data=False))
                return
            if self._rc_landing_active() or self.localization_failure_action in ("SAFE_STOP", "ALT_HOLD", "HOLD"):
                self.arm_pub.publish(Bool(data=False))
            else:
                self.land_pub.publish(Bool(data=True))
        elif elapsed < 2.0:
            if self.is_armed:
                self._publish_rc(1500, 1500, self._safe_stop_throttle(), 1500)
            self.arm_pub.publish(Bool(data=False))
        elif self.complete_requires_disarmed and self.is_armed and elapsed < 10.0:
            self._publish_rc(1500, 1500, self._safe_stop_throttle(), 1500)
            self.arm_pub.publish(Bool(data=False))
        else:
            self.get_logger().info("任务结束，重置回IDLE等待新任务")
            self.transition(State.IDLE)

    # ===== ABORT =====
    def _handle_abort(self):
        elapsed = self._elapsed()
        self._publish_strike_permission(False)
        # S1修复: 第一tick立即发送RC中立值，停止当前运动
        if elapsed < 0.3:
            self._publish_rc(1500, 1500, self.hover_pwm, 1500)
        # 第一tick立即重置mission计时，防止主循环超时检查循环触发ABORT
        if self.mission_start_time is not None or self.mission_started:
            self.mission_started = False
            self.mission_start_time = None
            self.get_logger().info(
                f"中止: 已清除mission计时器 (模式={self.current_mode}, "
                f"armed={self.is_armed}, alt={self.current_alt:.2f}m)"
            )

        if elapsed < 3.0:
            if self.is_armed:
                if self._rc_landing_active():
                    mode = "STABILIZE" if self.landing_strategy == "STABILIZE_RC" else "ALT_HOLD"
                    if self.current_mode != mode:
                        self.mode_pub.publish(String(data=mode))
                    self._publish_rc(1500, 1500, self._landing_descent_throttle(), 1500)
                else:
                    self.land_pub.publish(Bool(data=True))
                if elapsed > 1.0:
                    self.arm_pub.publish(Bool(data=False))
            self.get_logger().error(
                f"任务中止! t={elapsed:.1f}s 模式={self.current_mode} armed={self.is_armed}"
            )
        elif elapsed < 4.0:
            self.strike_perm_pub.publish(Bool(data=False))
        else:
            self.get_logger().info(
                f"中止完成，重置回IDLE等待新任务 "
                f"(模式={self.current_mode}, armed={self.is_armed}, alt={self.current_alt:.2f}m)"
            )
            self._ekf_abort_triggered = False  # 重置EKF中止标志
            self.transition(State.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = UAVMissionStateMachine()
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
