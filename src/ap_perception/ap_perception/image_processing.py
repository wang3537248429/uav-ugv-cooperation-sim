"""Bound image work without competing with the flight/physics processes."""

import math
import os

import cv2
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class LatestImageSubscription:
    """Keep one newest frame and process on a bounded single-executor timer.

    Receive and process callbacks run serially in rclpy's SingleThreadedExecutor.
    OpenCV gets an explicit thread budget; no Python thread or unbounded task
    queue is created for each incoming image.
    """

    def __init__(self, node, topic, process, default_fps=10.):
        node.declare_parameter('processing_fps', default_fps)
        node.declare_parameter('opencv_threads', max(1, int(os.environ.get('OPENCV_FOR_THREADS_NUM', '1'))))
        self.fps = float(node.get_parameter('processing_fps').value)
        threads = int(node.get_parameter('opencv_threads').value)
        if not math.isfinite(self.fps) or self.fps <= 0 or threads < 1:
            raise ValueError('processing_fps must be positive/finite and opencv_threads >= 1')
        cv2.setNumThreads(threads)
        self.latest = None
        self.process = process
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = node.create_subscription(Image, topic, self.receive, qos)
        self.timer = node.create_timer(1. / self.fps, self.tick)
        node.get_logger().info(f'Image processing <= {self.fps:g}Hz, latest frame only, OpenCV threads={threads}')

    def receive(self, message):
        self.latest = message

    def tick(self):
        message, self.latest = self.latest, None
        if message is not None:
            self.process(message)
