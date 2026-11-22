"""Bridge node defaults; deployment overrides belong in ROS YAML files."""

# 比赛任务统一由状态机发布 /ap/cmd/guided_position，
# 桥接节点只负责转发 GUIDED 位置/速度目标。
# EKF origin 仍需要一个固定全球参考点，供无 GPS 的 LOCAL_NED/GUIDED 初始化。
# 下列坐标是 ArduPilot SITL 的公开默认仿真原点，不是参赛者或设备位置。
ORIGIN_LAT = -35.363261
ORIGIN_LON = 149.165230
DEFAULT_ORIGIN_ALT_MSL = 584.0
DEFAULT_SITL_URL = "tcp:127.0.0.1:5760"


BRIDGE_PARAMETER_DEFAULTS = (
    ("connection_url", DEFAULT_SITL_URL),
    # Optional MAVLink fan-out for QGroundControl/Mission Planner.  Keep it
    # empty by default so the simulation remains a single ROS-controlled
    # vehicle; use udpout:127.0.0.1:14550 for a local GCS.
    ("gcs_output_url", ""),
    ("gcs_forward_commands", True),
    ("allow_hardware_connection", False),
    ("source_system", 191),
    ("source_component", 191),
    ("enable_external_nav", True),
    ("localization_backend", "external_nav"),
    ("vision_pose_cov_pos_xy", 0.02),
    ("vision_pose_cov_pos_z", 100.0),
    ("vision_pose_cov_vel", 0.1),
    ("vision_pose_cov_ang_yaw", 0.03),
    ("vision_pose_cov_ang_rp", 100.0),
    ("ap_viso_yaw_noise_rad", 0.2),
    ("set_ekf_origin_on_start", True),
    ("set_home_on_start", False),
    ("force_vision_flat_attitude", True),
    ("vision_yaw_source", "external_nav"),
    ("vision_yaw_attitude_max_age", 0.5),
    ("vision_pose_jump_guard_enabled", False),
    ("vision_pose_jump_max_m", 0.35),
    ("vision_pose_jump_max_mps", 1.0),
    ("vision_pose_jump_hold_timeout", 3.0),
    ("send_vision_speed_estimate", True),
    ("vision_speed_max_mps", 3.0),
    ("vision_speed_jump_mps", 8.0),
    ("vision_speed_min_dt", 0.02),
    ("vision_speed_max_dt", 0.5),
    ("vision_speed_lpf_alpha", 0.35),
    ("enable_native_optical_flow", False),
    ("optical_flow_input_topic", "/optical_flow/velocity"),
    ("optical_flow_quality_topic", "/optical_flow/quality"),
    ("distance_sensor_topic", "/range/down"),
    ("optical_flow_min_quality", 80),
    ("optical_flow_min_height", 0.05),
    ("optical_flow_max_height", 5.0),
    ("optical_flow_min_dt", 0.005),
    ("optical_flow_max_dt", 0.20),
    ("optical_flow_max_rate_rad_s", 8.0),
    ("optical_flow_sensor_id", 0),
    ("distance_sensor_id", 0),
    ("distance_sensor_min_m", 0.05),
    ("distance_sensor_max_m", 8.0),
    ("distance_sensor_orientation", 25),
    ("optical_flow_x_sign", 1.0),
    ("optical_flow_y_sign", 1.0),
    ("optical_flow_sensor_yaw_rad", 0.0),
    ("external_nav_diagnostics", True),
    ("external_nav_diagnostics_period", 2.0),
    # Barometric relative altitude is independent of GPS and complements the
    # Mid-360 scan matcher during the short floor/ceiling ambiguity at takeoff.
    ("external_nav_baro_z_fusion", False),
    ("external_nav_baro_z_max_age", 0.8),
    ("guided_attitude_climb_full_scale", 10.0),
    ("guided_attitude_hover_thrust", 0.5),
    ("guided_attitude_thrust_min", 0.47),
    ("guided_attitude_thrust_max", 0.515),
    ("guided_yaw_lock_enabled", True),
    ("guided_yaw_lock_rad", 0.0),
    ("invert_guided_position_y", False),
    ("invert_guided_velocity_y", False),
    ("ap_angle_max_cd", 600.0),
    ("ap_psc_angle_max_deg", 6.0),
    ("ap_guid_options", 8),
    ("ap_loit_speed_cms", 30.0),
    ("ap_loit_acc_max_cmss", 40.0),
    ("ap_wpnav_speed_cms", 40.0),
    ("ap_wpnav_speed_up_cms", 25.0),
    ("ap_wpnav_speed_dn_cms", 35.0),
    ("ap_wpnav_accel_cmss", 40.0),
    ("ap_wpnav_accel_z_cmss", 35.0),
    ("ap_psc_posxy_p", 0.5),
    ("ap_psc_velxy_p", 0.16),
    ("ap_psc_velxy_i", 0.01),
    ("ap_psc_velxy_d", 0.0),
    ("ap_mot_thst_hover", 0.50),
    ("nav_takeoff_min_altitude", 0.8),
    ("allow_low_alt_nav_takeoff", False),
    ("mavlink_read_period", 0.02),
    ("raw_controller_hz", 20),
    ("force_disarm", False),
    ("ekf_origin_retry_period", 2.0),
)
