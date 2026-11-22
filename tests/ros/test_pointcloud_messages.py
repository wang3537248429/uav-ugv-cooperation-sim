"""Real ROS point-cloud layouts and localization freshness behavior."""

from types import SimpleNamespace
import time
from unittest.mock import patch

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.time import Time

from ap_navigation.navigation_node import PointCloudNavigation
from ap_navigation.pointcloud import make_cloud, read_xyz
from ap_mission.uav_mission_state_machine import UAVMissionStateMachine
from ap_mavlink_bridge.mavlink_bridge_node import APMavlinkBridge
from ap_mission.mission_state import State


def test_organized_big_endian_cloud_with_row_padding():
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['>f4'] * 3,
                      'offsets': [4, 12, 20], 'itemsize': 24})
    storage = bytearray(2 * 56)
    values = np.ndarray((2, 2), dtype=dtype, buffer=storage, strides=(56, 24))
    for i, name in enumerate(('x', 'y', 'z')):
        values[name] = np.arange(4).reshape(2, 2) + i * 10
    message = PointCloud2(
        height=2, width=2, point_step=24, row_step=56, is_bigendian=True, data=bytes(storage))
    message.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                      for n, o in zip(('x', 'y', 'z'), (4, 12, 20))]
    np.testing.assert_array_equal(read_xyz(message), [[0, 10, 20], [1, 11, 21], [2, 12, 22], [3, 13, 23]])
    message.data = b'short'
    with pytest.raises(ValueError, match='truncated'):
        read_xyz(message)


def test_pointcloud_roundtrip_preserves_coordinates_and_frame():
    xyz = np.array([[1., 2., 3.], [-1., 0., 0.2]])
    message = make_cloud(xyz, Header(frame_id='uav_map'))
    assert message.header.frame_id == 'uav_map'
    np.testing.assert_allclose(read_xyz(message), xyz, atol=1e-7)


def test_missing_scan_revokes_ready_without_republishing_old_external_nav():
    ready_messages, status_messages = [], []
    harness = SimpleNamespace(
        good_scans=5, last_accepted=time.monotonic() - 3., last_input=time.monotonic() - 3.,
        stale_timeout=.7, reason='ok', mode='localization', rmse=.01, fitness=.9,
        mapper=SimpleNamespace(points=np.zeros((200, 3)), accepted_scans=10),
        ready_pub=SimpleNamespace(publish=ready_messages.append),
        status_pub=SimpleNamespace(publish=status_messages.append),
    )
    PointCloudNavigation.health(harness)
    assert ready_messages[-1].data is False
    assert harness.good_scans == 0
    assert harness.reason == 'cloud_stale'


@pytest.mark.parametrize('ned_z', [0., .05, -.01])
def test_ground_localization_does_not_use_noisy_baro_to_claim_liftoff(ned_z):
    harness = SimpleNamespace(
        enable_sim_truth_monitor=False, current_alt=.7,
        _navigation_position=lambda: (0., 0., ned_z),
    )
    assert UAVMissionStateMachine._nav_altitude(harness) == max(0., -ned_z)


def test_mission_velocity_uses_measurement_interval_when_rendering_is_slow():
    harness = SimpleNamespace(
        _last_external_nav_motion_sample=None, external_nav_velocity_lpf_alpha=1.,
        _external_nav_velocity=(0., 0., 0.), _external_nav_speed_xy=0., _external_nav_vertical_rate=0.,
    )
    first = PoseWithCovarianceStamped()
    first.header.stamp.sec = 1
    last = PoseWithCovarianceStamped()
    last.header.stamp.sec = 1
    last.header.stamp.nanosec = 100_000_000
    last.pose.pose.position.x = .1
    with patch('ap_mission.uav_mission_state_machine.time.time', side_effect=[1000., 1002.]):
        UAVMissionStateMachine.external_nav_pose_cb(harness, first)
        UAVMissionStateMachine.external_nav_pose_cb(harness, last)
    assert harness._external_nav_velocity[0] == pytest.approx(1.)
    assert harness._last_external_nav_pose_time == 1002.


def test_mavlink_velocity_and_timestamp_preserve_sensor_clock():
    sent = []
    harness = SimpleNamespace(
        send_vision_speed_estimate=True, _mavlink_ready=lambda: True,
        _last_vision_speed_sample=(1., 0., 0., 0.), _vision_speed_lpf=(0., 0., 0.),
        vision_speed_min_dt=.02, vision_speed_max_dt=.5, vision_speed_jump_mps=8.,
        vision_speed_max_mps=3., vision_speed_lpf_alpha=1., cov_vel=.1,
        master=SimpleNamespace(mav=SimpleNamespace(vision_speed_estimate_send=lambda *a, **kw: sent.append(a))),
    )
    with patch('ap_mavlink_bridge.mavlink_bridge_node.time.monotonic', return_value=1002.):
        APMavlinkBridge._send_vision_speed_if_available(harness, .1, 0., 0., sample_time=1.1)
    assert sent[0][0] == 1_100_000
    assert sent[0][1] == pytest.approx(1.)
    assert harness._last_vision_speed_sent[0] == 1002.


def test_native_takeoff_is_not_cancelled_by_a_position_command():
    transitions, positions = [], []
    harness = SimpleNamespace(
        _elapsed=lambda: 5., _nav_altitude=lambda: .4, target_altitude=1., takeoff_success_altitude=.22,
        takeoff_start_alt=0., _takeoff_use_guided_attitude_fallback=False, _takeoff_use_rc_fallback=False,
        current_mode='GUIDED', _rc_released_for_guided=True, _takeoff_command_sent=True,
        native_guided_takeoff=True, _navigation_vertical_rate=lambda: .1, _takeoff_xy=(0., 0.),
        transition=transitions.append, _publish_guided_position=lambda *args: positions.append(args),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None),
    )
    UAVMissionStateMachine._handle_guided_takeoff(harness)
    assert not positions and not transitions
    harness._nav_altitude = lambda: .95
    UAVMissionStateMachine._handle_guided_takeoff(harness)
    assert transitions == [State.HOVER_STABLE]
    assert not positions


def test_home_reference_requires_ack_and_is_never_reset_while_armed():
    sent, ready = [], []
    harness = SimpleNamespace(
        enable_external_nav=True, set_ekf_origin_on_start=True, _mavlink_ready=lambda: True,
        home_ready_pub=SimpleNamespace(publish=lambda msg: ready.append(msg.data)),
        _latest_home=None, _latest_global_origin=(99., -35., 149., 584.), _origin_set=True,
        set_home_on_start=True, is_armed=False, _last_home_request=0.,
        _last_origin_request=100., ekf_origin_retry_period=2.,
        master=SimpleNamespace(target_system=1, mav=SimpleNamespace(command_int_send=lambda *args: sent.append(args))),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None), _request_home_position=lambda: None,
    )
    with patch('ap_mavlink_bridge.mavlink_bridge_node.time.monotonic', return_value=100.):
        APMavlinkBridge._origin_tick(harness)
        assert len(sent) == 1 and ready[-1] is False
        assert sent[-1][-3:] == (-350000000, 1490000000, 584.)
        harness.is_armed = True
        harness._last_home_request = 0.
        APMavlinkBridge._origin_tick(harness)
        assert len(sent) == 1
        harness._latest_home = (100., -35., 149., 584.)
        APMavlinkBridge._origin_tick(harness)
        assert ready[-1] is True


def test_native_flight_preflight_waits_for_home_ack():
    transitions = []
    harness = SimpleNamespace(
        _elapsed=lambda: 1., is_connected=True, get_parameter=lambda name: SimpleNamespace(value=0.),
        battery_voltage=16., ekf_ok=True, _navigation_is_ready=lambda: True, localization_backend='external_nav',
        current_mode='GUIDED', native_guided_takeoff=True, fcu_home_ready=False,
        get_logger=lambda: SimpleNamespace(info=lambda msg: None), transition=transitions.append,
    )
    UAVMissionStateMachine._handle_check_system(harness)
    assert not transitions
    harness.fcu_home_ready = True
    UAVMissionStateMachine._handle_check_system(harness)
    assert transitions == [State.WAIT_START]


def test_lost_early_origin_ack_is_retried_before_home_confirmation():
    requested = []
    harness = SimpleNamespace(
        enable_external_nav=True, set_ekf_origin_on_start=True, _mavlink_ready=lambda: True,
        home_ready_pub=SimpleNamespace(publish=lambda msg: None), _latest_home=None,
        _latest_global_origin=(1., -35., 149., 584.), _origin_set=True, set_home_on_start=True,
        _request_message_interval=lambda *args: None, _request_home_position=lambda: None,
        _last_origin_request=0., ekf_origin_retry_period=2., _latest_local_pos=None,
        is_armed=False,
    )

    def request_origin():
        requested.append(True)
        harness._last_origin_request = 100.
    harness.set_ekf_origin = request_origin
    with patch('ap_mavlink_bridge.mavlink_bridge_node.time.monotonic', return_value=100.):
        APMavlinkBridge._origin_tick(harness)
    assert requested == [True]
    assert not harness._origin_set


def test_native_landing_requires_ground_contact_and_level_settle_before_success():
    armed_messages, transitions = [], []
    harness = SimpleNamespace(
        _nav_altitude=lambda: .5, _navigation_vertical_rate=lambda: 0.,
        fcu_landed=True, _last_landed_time=100., is_armed=True, current_mode='LAND',
        _landing_touchdown_since=None, _landing_settled_since=None,
        arm_pub=SimpleNamespace(publish=armed_messages.append),
        attitude_rpy=(0., 0., 0.), _mission_success=True,
        _elapsed=lambda: 10., transition=transitions.append,
    )
    with patch('ap_mission.uav_mission_state_machine.time.time', return_value=100.):
        UAVMissionStateMachine._handle_native_landing(harness)
    assert not armed_messages  # Even an early landed flag cannot cut motors in the air.
    harness._nav_altitude = lambda: 0.
    with patch('ap_mission.uav_mission_state_machine.time.time', return_value=100.):
        UAVMissionStateMachine._handle_native_landing(harness)
    with patch('ap_mission.uav_mission_state_machine.time.time', return_value=101.1):
        UAVMissionStateMachine._handle_native_landing(harness)
    assert armed_messages[-1].data is False
    harness.is_armed = False
    with patch('ap_mission.uav_mission_state_machine.time.time', return_value=102.):
        UAVMissionStateMachine._handle_native_landing(harness)
    assert not transitions
    harness._last_landed_time = 104.1
    with patch('ap_mission.uav_mission_state_machine.time.time', return_value=104.1):
        UAVMissionStateMachine._handle_native_landing(harness)
    assert transitions == [State.COMPLETE]


@pytest.mark.parametrize('next_task', ['RETURN_TAKEOFF_ZONE', 'BALLOON_ALT'])
def test_task_resume_keeps_intermediate_route_points_instead_of_skipping_them(next_task):
    route = [(0., 0., 1., 'PREVIOUS_TASK'), (1., .4, 1., 'BEND'), (2., 0., 1., next_task)]
    transitions = []
    harness = SimpleNamespace(
        _wpt_waypoints=route, _wpt_index=1, waypoint_timeout=60.,
        _find_waypoint_index=lambda label: 2 if label == next_task else None,
        get_logger=lambda: SimpleNamespace(error=lambda msg: None), transition=transitions.append,
    )
    assert UAVMissionStateMachine._resume_waypoints_through_label(harness, next_task)
    assert harness._wpt_index == 1 and harness._guided_hold_target == route[1]
    harness._wpt_index = 3
    assert not UAVMissionStateMachine._resume_waypoints_through_label(harness, next_task)
    assert transitions == [State.ABORT]


def test_rviz_markers_include_every_point_of_an_extended_route():
    published = []
    route = [(float(i), 0., 1., f'POINT_{i}') for i in range(12)]
    harness = SimpleNamespace(
        _wpt_waypoints=route, _wpt_index=1, waypoint_marker_frame='uav_map',
        get_clock=lambda: SimpleNamespace(now=lambda: Time()),
        waypoint_pub=SimpleNamespace(publish=published.append),
    )
    UAVMissionStateMachine._publish_waypoints(harness)
    labels = [m.text for m in published[0].markers if m.ns == 'waypoint_labels']
    assert labels == [f'WP{i} POINT_{i}' for i in range(12)]
