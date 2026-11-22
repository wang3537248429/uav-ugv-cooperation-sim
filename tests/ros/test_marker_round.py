"""Mission triggers randomize a marker once, including repeated start messages."""

from types import SimpleNamespace
from unittest.mock import patch

from std_msgs.msg import Bool, String

from ap_simulation.sim_camera_node import SimCameraNode


def test_round_marker_is_stable_until_reset_or_terminal_mission_state():
    published = []
    camera = SimpleNamespace(
        round_active=False, randomize_marker_on_start=True, aruco_id=0, marker_count=50,
        scene={}, marker_pub=SimpleNamespace(publish=lambda msg: published.append(msg.data)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    with patch('ap_simulation.sim_camera_node.next_marker_id', side_effect=[17, 23, 8]) as select:
        for _ in range(5):
            SimCameraNode._start_cb(camera, Bool(data=True))
        assert published == [17] and camera.scene['aruco_id'] == 17
        assert select.call_count == 1
        SimCameraNode._start_cb(camera, Bool(data=False))
        SimCameraNode._start_cb(camera, Bool(data=True))
        assert published == [17, 23] and camera.scene['aruco_id'] == 23
        SimCameraNode._state_cb(camera, String(data='COMPLETE'))
        SimCameraNode._start_cb(camera, Bool(data=True))
        assert published == [17, 23, 8] and camera.scene['aruco_id'] == 8
