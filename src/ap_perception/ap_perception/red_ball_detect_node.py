#!/usr/bin/env python3
"""
红气球视觉识别节点
==================
输入: 仿真下视相机 (640x480)
输入: /camera/down/image_raw (sensor_msgs/Image)
输出: /vision/red_ball (std_msgs/Float32MultiArray)
  data[0] = found          (0/1)
  data[1] = cx_error       (-1到1, 0=居中)
  data[2] = cy_error       (-1到1, 0=居中)
  data[3] = area_ratio     (红色面积占比)
  data[4] = circularity    (圆度)

算法: 移植自官方 ShootBall3.py:38-54 calc_centroid()
  - HSV双阈值红色检测 (H: 0-10 + 170-180)
  - 形态学开闭运算去噪
  - 最大轮廓提取 + 面积过滤 + 圆度判断
  - 连续帧确认 (防误检)

调参: 所有关键参数均为 ROS 2 参数, 可通过命令行或 YAML 动态调整
  ros2 param set /red_ball_detect_node hsv_s_min 25
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from .image_processing import LatestImageSubscription


class RedBallDetectNode(Node):
    """红气球检测节点"""

    def __init__(self):
        super().__init__("red_ball_detect_node")

        # ===== 检测参数 (可动态调整) =====
        self.declare_parameter("confirm_frames", 5)
        self.declare_parameter("min_area", 100)
        self.declare_parameter("min_circularity", 0.3)
        self.declare_parameter("image_topic", "/camera/down/image_raw")

        # HSV 红色双阈值 (可在比赛现场快速调参)
        self.declare_parameter("hsv_lower_red1", [0, 80, 80])
        self.declare_parameter("hsv_upper_red1", [10, 255, 255])
        self.declare_parameter("hsv_lower_red2", [170, 80, 80])
        self.declare_parameter("hsv_upper_red2", [180, 255, 255])

        # 形态学参数
        self.declare_parameter("morph_open_size", 5)
        self.declare_parameter("morph_close_size", 5)

        # 缓存参数值 (避免每帧都调用 get_parameter)
        self._cache_params()

        # 连续帧确认
        self.detected_count = 0

        # CV Bridge
        self.bridge = CvBridge()

        # 发布者
        self.result_pub = self.create_publisher(
            Float32MultiArray, "/vision/red_ball", 10
        )

        # 订阅者
        self.image_pipeline = LatestImageSubscription(self, self._param('image_topic'), self.image_callback)

        self.get_logger().info("红气球检测节点已启动 (HSV阈值可动态调整)")

    def _param(self, name):
        return self.get_parameter(name).value

    def _cache_params(self):
        """缓存全部参数值"""
        self.confirm_frames = self._param("confirm_frames")
        self.min_area = self._param("min_area")
        self.min_circularity = self._param("min_circularity")
        self.hsv_lower1 = np.array(self._param("hsv_lower_red1"))
        self.hsv_upper1 = np.array(self._param("hsv_upper_red1"))
        self.hsv_lower2 = np.array(self._param("hsv_lower_red2"))
        self.hsv_upper2 = np.array(self._param("hsv_upper_red2"))
        self.kernel_open = np.ones((self._param("morph_open_size"),) * 2, np.uint8)
        self.kernel_close = np.ones((self._param("morph_close_size"),) * 2, np.uint8)

    def image_callback(self, msg: Image):
        # 每30帧刷新一次参数缓存 (允许运行时动态调参)
        frame_count = getattr(self, "_frame_count", 0) + 1
        self._frame_count = frame_count
        if frame_count % 30 == 0:
            self._cache_params()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # M9修复: 检测函数加异常保护, 单帧畸形数据不导致节点崩溃
        try:
            found, cx_err, cy_err, area_r, circ = self._detect(frame)
        except Exception as e:
            self.get_logger().error(f"红球检测异常: {e}")
            return

        # 连续帧确认: 使用非对称积分器 (发现+2, 丢失-1)
        # 避免闪烁时 (1帧检测/1帧丢失) 始终无法确认的问题
        if found:
            self.detected_count = min(self.confirm_frames, self.detected_count + 2)
        else:
            self.detected_count = max(0, self.detected_count - 1)

        confirmed = self.detected_count >= self.confirm_frames

        result_msg = Float32MultiArray()
        result_msg.data = [
            1.0 if confirmed else 0.0,
            cx_err if confirmed else 0.0,
            cy_err if confirmed else 0.0,
            area_r if confirmed else 0.0,
            circ if confirmed else 0.0,
        ]
        self.result_pub.publish(result_msg)

    def _detect(self, frame):
        """
        HSV 双阈值红色气球检测。
        返回: (found, cx_error, cy_error, area_ratio, circularity)
        """
        if frame is None:
            return False, 0.0, 0.0, 0.0, 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色双阈值
        mask1 = cv2.inRange(hsv, self.hsv_lower1, self.hsv_upper1)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形态学去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

        # 最大轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, 0.0, 0.0

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        h, w = frame.shape[:2]

        if area < self.min_area:
            return False, 0.0, 0.0, 0.0, 0.0

        # 圆度判断
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False, 0.0, 0.0, 0.0, 0.0
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < self.min_circularity:
            return False, 0.0, 0.0, 0.0, 0.0

        # 中心偏差 (归一化到 -1~1)
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return False, 0.0, 0.0, 0.0, 0.0
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        cx_error = (cx - w / 2) / (w / 2)
        cy_error = (cy - h / 2) / (h / 2)
        area_ratio = area / float(w * h)

        return True, cx_error, cy_error, area_ratio, circularity


def main(args=None):
    rclpy.init(args=args)
    node = RedBallDetectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
