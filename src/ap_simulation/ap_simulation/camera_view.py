"""A small live ROS camera window; Qt and ROS share one bounded event loop."""

import signal
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class CameraWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Downward Camera — Live /camera/down/image_raw')
        self.resize(640, 520)
        self.move(30, 450)
        self.picture = QLabel('Waiting for camera…')
        self.picture.setAlignment(Qt.AlignCenter)
        self.picture.setMinimumSize(320, 240)
        self.picture.setStyleSheet('background: #151719; color: white;')
        self.status = QLabel('/camera/down/image_raw · simulated downward camera')
        layout = QVBoxLayout(self)
        layout.addWidget(self.picture, 1)
        layout.addWidget(self.status)
        self.latest = None
        self.marker_id = '?'
        self.last_frame = time.monotonic()
        self.subscription = node.create_subscription(
            Image, '/camera/down/image_raw', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.marker_subscription = node.create_subscription(
            Int32, '/simulation/aruco_id', self.marker_received,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    def receive(self, message):
        if message.encoding != 'bgr8' or len(message.data) < message.height * message.step:
            self.status.setText('Unsupported/truncated camera frame')
            return
        self.latest = QImage(bytes(message.data), message.width, message.height,
                             message.step, QImage.Format_BGR888).copy()
        self.last_frame = time.monotonic()

    def marker_received(self, message):
        self.marker_id = str(message.data)

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0.)
        if self.latest is not None:
            frame, self.latest = self.latest, None
            self.picture.setPixmap(QPixmap.fromImage(frame).scaled(
                self.picture.size(), Qt.KeepAspectRatio, Qt.FastTransformation))
            self.status.setText(
                f'LIVE · {frame.width()} × {frame.height()} · Downward camera · ArUco ID {self.marker_id}')
        elif time.monotonic() - self.last_frame > 1.:
            self.status.setText('Camera stream stale — waiting for a new frame')


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = Node('camera_view')
    window = CameraWindow(node)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
