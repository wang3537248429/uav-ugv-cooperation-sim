"""Pure-Python regression tests for the competition simulation contract."""

import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np


from ap_mission.competition_route import COMPETITION_WAYPOINTS_NED
from ap_simulation.competition_paths import (
    UGV_GATE_PROGRESS,
    UGV_PATH_GAZEBO,
    UGV_STRIKE_GAZEBO_X,
    UGV_STRIKE_GAZEBO_Y,
    gazebo_xy_to_local_ned,
)
from ap_simulation.scene_generator import generate_camera_image


ROOT = Path(__file__).resolve().parents[2]


def test_uav_strike_waypoint_matches_ugv_final_stop():
    strike = next(
        (x, y)
        for x, y, _altitude, label in COMPETITION_WAYPOINTS_NED
        if label == "STRIKE_ZONE"
    )
    assert UGV_PATH_GAZEBO[-1] == (
        UGV_STRIKE_GAZEBO_X,
        UGV_STRIKE_GAZEBO_Y,
    )
    assert strike == gazebo_xy_to_local_ned(*UGV_PATH_GAZEBO[-1])


def test_route_has_wall_clearance_and_ordered_gates():
    # The south wall is at Gazebo y=-2.50. Keep more than one UAV half-width.
    assert abs(-2.50 - UGV_STRIKE_GAZEBO_Y) >= 0.33
    assert all(
        first < second
        for first, second in zip(UGV_GATE_PROGRESS, UGV_GATE_PROGRESS[1:])
    )
    assert 0.0 < UGV_GATE_PROGRESS[0] < UGV_GATE_PROGRESS[-1] < 1.0
    return_zone = next(
        x
        for x, _y, _altitude, label in COMPETITION_WAYPOINTS_NED
        if label == "RETURN_TAKEOFF_ZONE"
    )
    assert 0.30 <= return_zone <= 0.75
    scan_zone = next(
        (x, y)
        for x, y, _altitude, label in COMPETITION_WAYPOINTS_NED
        if label == "SCAN_ZONE"
    )
    assert scan_zone == (3.80, 0.00)


def test_synthetic_camera_contains_detectable_aruco_from_safe_scan_position():
    image = generate_camera_image(3.6, 0.0, 1.0)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    assert ids is not None
    assert 0 in ids.flatten().tolist()


def test_aruco_detection_tolerates_optical_flow_position_error():
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    )
    for drone_x in (3.6, 3.8, 4.0):
        image = generate_camera_image(drone_x, 0.0, 1.0)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _corners, ids, _rejected = detector.detectMarkers(gray)
        assert ids is not None and 0 in ids.flatten().tolist()


def test_velocity_nav_recovers_endpoint_overshoot_without_mode_switch():
    mission_source = (
        ROOT
        / "src"
        / "ap_mission"
        / "ap_mission"
        / "uav_mission_state_machine.py"
    ).read_text()
    assert "endpoint_along_error = err_x * seg_ux + err_y * seg_uy" in mission_source
    assert "if endpoint_along_error < 0.0:" in mission_source
    arrival_block = mission_source.split(
        "if dist_xy < WAYPOINT_REACHED_RADIUS", 1
    )[1].split("# 超时处理", 1)[0]
    assert 'f"{label}_ARRIVAL_BRAKE"' in arrival_block
    assert "if use_velocity_nav:" in arrival_block
    assert "bypass_slew=True" in arrival_block


def test_synthetic_camera_contains_red_balloon_at_strike_zone():
    strike_x, strike_y = gazebo_xy_to_local_ned(
        UGV_STRIKE_GAZEBO_X,
        UGV_STRIKE_GAZEBO_Y,
    )
    image = generate_camera_image(strike_x, strike_y, 0.8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, np.array([0, 120, 100]), np.array([10, 255, 255]))
    assert cv2.countNonZero(red) > 1000


def test_uav_model_uses_vendored_px4_geometry_and_competition_scale():
    model_dir = (
        ROOT
        / "src"
        / "ap_simulation"
        / "models"
        / "competition_x500"
    )
    template_root = ET.parse(model_dir / "model.sdf.in").getroot()
    px4_include = next(
        include
        for include in template_root.iter("include")
        if include.findtext("uri") == "model://x500_base"
    )
    assert px4_include.findtext("scale").split() == ["0.67", "0.67", "0.67"]

    generated_root = ET.parse(model_dir / "model.sdf").getroot()
    rotor_0 = next(
        link
        for link in generated_root.iter("link")
        if link.get("name") == "rotor_0"
    )
    rotor_pose = [float(value) for value in rotor_0.findtext("pose").split()]
    assert rotor_pose[:2] == [0.11658, -0.11658]
    base_link = next(
        link
        for link in generated_root.iter("link")
        if link.get("name") == "base_link"
    )
    assert base_link.findtext("velocity_decay/linear") == "0.4"
    assert base_link.findtext("velocity_decay/angular") == "0.05"
    guard_collisions = [
        collision
        for collision in base_link.findall("collision")
        if collision.get("name", "").startswith(
            "competition_prop_guard_collision_"
        )
    ]
    assert len(guard_collisions) == 4
    guard_centres = {
        tuple(float(value) for value in collision.findtext("pose").split()[:2])
        for collision in guard_collisions
    }
    assert guard_centres == {
        (0.11658, -0.11658),
        (-0.11658, 0.11658),
        (0.11658, 0.11658),
        (-0.11658, -0.11658),
    }
    for collision in guard_collisions:
        assert collision.findtext("./geometry/cylinder/radius") == "0.048"
        assert collision.findtext("./geometry/cylinder/length") == "0.01"
    for index in range(4):
        rotor = next(
            link
            for link in generated_root.iter("link")
            if link.get("name") == f"rotor_{index}"
        )
        assert rotor.findall("collision") == []
    assert any(
        mesh.findtext("uri", "").startswith("model://x500_base/")
        for mesh in generated_root.iter("mesh")
    )
    motor_directions = {
        plugin.findtext("linkName"): plugin.findtext("turningDirection")
        for plugin in generated_root.iter("plugin")
        if plugin.get("filename") == "gz-sim-multicopter-motor-model-system"
    }
    assert motor_directions == {
        "rotor_0": "ccw",
        "rotor_1": "ccw",
        "rotor_2": "cw",
        "rotor_3": "cw",
    }

    px4_base = (
        ROOT
        / "src"
        / "ap_simulation"
        / "models"
        / "x500_base"
    )
    assert (px4_base / "LICENSE").is_file()
    assert (px4_base / "meshes" / "NXP-HGD-CF.dae").is_file()
    assert "BSD 3-Clause License" in (px4_base / "LICENSE").read_text()


def test_ardupilot_pwm_matches_px4_x500_rotor_order():
    params = (
        ROOT / "src" / "ap_simulation" / "config" / "sim_params.yaml"
    ).read_text()
    bridge_source = (
        ROOT
        / "src"
        / "ap_simulation"
        / "ap_simulation"
        / "motor_actuator_bridge.py"
    ).read_text()

    # PX4 rotor0..3 and ArduPilot Quad-X MotorDef array indices are both
    # FR(CCW), RL(CCW), FL(CW), RR(CW). MotorDef's final integer is only its
    # motor-test order and must not be treated as a PWM permutation.
    assert "motor_order: [0, 1, 2, 3]" in params
    assert 'declare_parameter("motor_order", [0, 1, 2, 3])' in bridge_source
    assert "回退为 [0, 1, 2, 3]" in bridge_source
    assert (
        "ArduPilot MotorDef is only the motor-test order"
        in bridge_source
    )


def test_rviz_frames_have_an_explicit_ned_to_display_transform():
    rviz_config = (
        ROOT / "src" / "ap_simulation" / "rviz" / "competition.rviz"
    ).read_text()
    launch_source = (
        ROOT / "src" / "ap_simulation" / "launch" / "sim_all.launch.py"
    ).read_text()
    marker_source = (
        ROOT
        / "src"
        / "ap_simulation"
        / "ap_simulation"
        / "competition_field_markers.py"
    ).read_text()

    assert "Fixed Frame: uav_map" in rviz_config
    assert '"--frame-id", "uav_map"' in launch_source
    assert '"--child-frame-id", "local_ned"' in launch_source
    transform_block = launch_source.split(
        "rviz_frame_transform = Node(", 1
    )[1].split("\n    )", 1)[0]
    assert "condition=" not in transform_block
    assert 'declare_parameter("frame_id", "uav_map")' in marker_source


def test_launch_isolates_gazebo_transport_from_other_projects():
    launch_source = (
        ROOT
        / "src"
        / "ap_simulation"
        / "launch"
        / "sim_all.launch.py"
    ).read_text()

    assert '"gz_partition"' in launch_source
    assert 'default_value="competition_ws"' in launch_source
    assert (
        'SetEnvironmentVariable("GZ_PARTITION", gz_partition)'
        in launch_source
    )
    assert 'cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world]' in launch_source
    assert (
        'cmd=["gz", "sim", "-g", "--gui-config", gz_gui_config]'
        in launch_source
    )
    gui_config = (
        ROOT / "src" / "ap_simulation" / "config" / "gz_gui.config"
    ).read_text()
    assert "<camera_pose>-8 -6 9 0 0.73 0.64</camera_pose>" in gui_config


def test_sim_takeoff_avoids_ardupilot_guided_submode_internal_error():
    params = (
        ROOT / "src" / "ap_simulation" / "config" / "sim_params.yaml"
    ).read_text()

    assert "set_ekf_origin_on_start: false" in params
    assert "use_guided_attitude_takeoff_fallback: false" in params
    assert "allow_takeoff_rc_fallback: true" in params
    assert "prefer_takeoff_rc_fallback: true" in params
    assert 'guided_nav_control_mode: "position"' in params
    assert 'guided_position_strategy: "direct"' in params
    assert "guided_nav_speed_max: 0.06" in params
    assert "guided_nav_cross_track_stop: 0.40" in params
    assert "guided_nav_cross_track_speed_max: 0.05" in params
    assert "guided_velocity_brake_speed_xy: 0.22" in params
    assert "ap_psc_posxy_p: 0.15" in params
    assert "ap_psc_velxy_p: 1.20" in params
    assert "ap_psc_velxy_i: 0.10" in params
    assert "ap_psc_velxy_d: 0.05" in params
    assert "vision_speed_lpf_alpha: 0.75" in params
    assert "external_nav_velocity_lpf_alpha: 0.75" in params
    assert "ap_wpnav_speed_cms: 10.0" in params
    assert "ap_wpnav_accel_cmss: 5.0" in params
    assert "guided_position_brake_speed_xy: 0.35" in params
    assert "guided_position_brake_resume_speed_xy: 0.20" in params
    assert "waypoint_reached_speed_xy_max: 0.12" in params
    assert "post_puncture_home_brake_radius: 1.80" in params
    assert "post_puncture_home_brake_damping: 0.80" in params
    mission_source = (
        ROOT
        / "src"
        / "ap_mission"
        / "ap_mission"
        / "uav_mission_state_machine.py"
    ).read_text()
    assert "speed = min(speed, self.post_puncture_home_speed_max)" in mission_source

    mavlink_source = (
        ROOT
        / "src"
        / "ap_mavlink_bridge"
        / "ap_mavlink_bridge"
        / "mavlink_bridge_node.py"
    ).read_text()
    assert mavlink_source.count(
        "无GPS原点/HOME_POSITION, 拒绝切换LAND"
    ) == 2
