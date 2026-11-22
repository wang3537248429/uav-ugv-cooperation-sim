#!/usr/bin/env python3
"""
仿真下视相机节点 — 针孔透视投影合成
订阅 /sim/drone_pose_ned (Gazebo真值本地NED) 获取无人机位姿 → 透视投影合成场景图像

发布: /camera/down/image_raw (sensor_msgs/Image, bgr8, 640x480)
"""

import rclpy
import cv2
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge
from .scene_generator import generate_camera_image, get_default_scene
from .competition_paths import balloon_pose_ned_scene
from .random_marker import next_marker_id


class SimCameraNode(Node):

    def __init__(self):
        super().__init__("sim_camera_node")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 15.0)
        self.declare_parameter("frame_id", "camera_down")
        self.declare_parameter("balloon_x", 0.0)
        self.declare_parameter("balloon_y", -1.8)
        self.declare_parameter("aruco_visible", True)
        self.declare_parameter("aruco_id", 0)
        self.declare_parameter('randomize_marker_on_start', True)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco_size", 0.15)
        self.declare_parameter("pose_topic", "/sim/drone_pose_ned")
        self.declare_parameter("track_ugv_balloon", True)
        self.declare_parameter("ugv_progress_topic", "/mission/ugv_progress")
        self.declare_parameter("balloon_punctured_topic", "/mission/balloon_punctured")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.framerate = float(self.get_parameter("framerate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.balloon_x = float(self.get_parameter("balloon_x").value)
        self.balloon_y = float(self.get_parameter("balloon_y").value)
        self.aruco_visible = bool(self.get_parameter("aruco_visible").value)
        self.aruco_id = int(self.get_parameter("aruco_id").value)
        self.aruco_dictionary = self.get_parameter("aruco_dictionary").value
        self.aruco_size = float(self.get_parameter("aruco_size").value)
        self.pose_topic = self.get_parameter("pose_topic").value
        self.track_ugv_balloon = bool(self.get_parameter("track_ugv_balloon").value)
        self.ugv_progress_topic = self.get_parameter("ugv_progress_topic").value
        self.balloon_punctured_topic = self.get_parameter("balloon_punctured_topic").value

        # 无人机位姿 (NED: x=北, y=东, z=下)
        self.drone_n = 0.0
        self.drone_e = 0.0
        self.drone_alt = 1.0  # 默认1m高度

        self.bridge = CvBridge()
        cv2.setNumThreads(max(1, int(os.environ.get('OPENCV_FOR_THREADS_NUM', '1'))))
        self.pub = self.create_publisher(
            Image, '/camera/down/image_raw', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scene = get_default_scene()
        self.scene["balloon_world"] = (self.balloon_x, self.balloon_y, self.scene["balloon_world"][2])
        self.scene["aruco_visible"] = self.aruco_visible
        self.scene["aruco_id"] = self.aruco_id
        self.scene["aruco_dictionary"] = self.aruco_dictionary
        self.scene["aruco_size"] = self.aruco_size
        self.scene["balloon_visible"] = True
        self.ugv_progress = 0.0
        self.round_active = False
        self.randomize_marker_on_start = bool(self.get_parameter('randomize_marker_on_start').value)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.aruco_dictionary))
        self.marker_count = len(dictionary.bytesList)
        self.marker_pub = self.create_publisher(
            Int32, '/simulation/aruco_id', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_pub.publish(Int32(data=self.aruco_id))
        self.create_subscription(Bool, '/mission/start', self._start_cb, 10)
        self.create_subscription(String, '/mission/uav_state', self._state_cb, 10)

        # 订阅 Gazebo 真值位姿, 避免 /ap/local_pos EKF 漂移反向驱动仿真传感器
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        if self.track_ugv_balloon:
            self.create_subscription(Float32, self.ugv_progress_topic, self._ugv_progress_cb, 10)
        self.create_subscription(Bool, self.balloon_punctured_topic, self._balloon_punctured_cb, 10)

        self.timer = self.create_timer(1.0 / self.framerate, self.publish_image)

        self.get_logger().info(
            f"仿真相机(透视): {self.width}x{self.height} @ {self.framerate}Hz "
            f"HFOV=60°, 订阅{self.pose_topic}获取位姿, "
            f"balloon_track={'UGV' if self.track_ugv_balloon else 'fixed'}"
        )

    def _pose_cb(self, msg: PoseStamped):
        """接收 Gazebo 真值 NED 位姿。"""
        pos = msg.pose.position
        self.drone_n = pos.x    # NED北
        self.drone_e = pos.y    # NED东
        self.drone_alt = -pos.z if pos.z < 0 else pos.z  # NED下→正高度

    def _start_cb(self, message):
        if not message.data:
            self.round_active = False
            return
        if self.round_active:
            return
        self.round_active = True
        if self.randomize_marker_on_start:
            self.aruco_id = next_marker_id(self.aruco_id, self.marker_count)
            self.scene['aruco_id'] = self.aruco_id
            self.marker_pub.publish(Int32(data=self.aruco_id))
            self.get_logger().info(f'本轮 ArUco ID={self.aruco_id}，任务内保持不变')

    def _state_cb(self, message):
        if message.data in ('COMPLETE', 'ABORT', 'IDLE'):
            self.round_active = False

    def _ugv_progress_cb(self, msg: Float32):
        self.ugv_progress = max(0.0, min(1.0, float(msg.data)))
        self.scene["balloon_world"] = balloon_pose_ned_scene(self.ugv_progress)

    def _balloon_punctured_cb(self, msg: Bool):
        self.scene["balloon_visible"] = not bool(msg.data)

    def publish_image(self):
        # scene_generator 使用与状态机一致的本地NED平面坐标:
        # x=North/前进, y=East/右向, z=Up。
        drone_x = self.drone_n
        drone_y = self.drone_e

        img = generate_camera_image(
            drone_x=drone_x,
            drone_y=drone_y,
            drone_alt=self.drone_alt,
            scene=self.scene,
            width=self.width,
            height=self.height,
        )

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimCameraNode()
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
