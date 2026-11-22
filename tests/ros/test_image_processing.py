"""Exercise image queue freshness/rate limits with actual ROS transport."""

import time
from types import SimpleNamespace, MethodType

import cv2
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from ap_perception.image_processing import LatestImageSubscription
from ap_perception.code_detect_node import CodeDetectNode
import numpy as np


def test_burst_images_keep_latest_frame_without_reprocessing_or_unbounded_threads():
    context = Context()
    context.init()
    node = Node('image_queue_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    processed = []
    pipeline = LatestImageSubscription(node, '/test/image_burst',
                                       lambda msg: processed.append(msg.header.stamp.nanosec), default_fps=8.)
    publisher = node.create_publisher(
        Image, '/test/image_burst', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)

    try:
        deadline = time.monotonic() + 2.
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert publisher.get_subscription_count() == 1
        for sequence in range(100):
            message = Image()
            message.header.stamp.nanosec = sequence
            publisher.publish(message)
        spin_for(.4)
        assert processed[-1] == 99
        assert len(processed) <= 3
        count = len(processed)
        spin_for(.3)
        assert len(processed) == count  # A held camera frame is not a new observation.
        assert pipeline.latest is None
        assert cv2.getNumThreads() == 1
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_downsampled_aruco_keeps_id_and_original_camera_coordinates():
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 80)
    frame[200:280, 280:360] = marker[:, :, None]
    harness = SimpleNamespace(
        detection_max_width=320, detector=cv2.aruco.ArucoDetector(dictionary),
        target_id=0, accept_any_id=False, last_result='', detection_count=0,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _save_detection_image=lambda *args: None,
    )
    harness._select_detection = MethodType(CodeDetectNode._select_detection, harness)
    annotated = frame.copy()
    found, code = CodeDetectNode._detect_aruco(harness, frame, annotated)
    assert found and code == 'ARUCO:0'
    np.testing.assert_array_equal(annotated[240, 320], [0, 255, 0])
