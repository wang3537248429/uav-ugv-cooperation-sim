#!/usr/bin/env python3
"""
红色方框检测节点
================
从下视相机图像中检测红色方框障碍（比赛任务：穿越方框）。
算法 100% 移植自官方 CrossRing3.py:86-110 detect_red_square()。

检测流程:
  1. HSV 双阈值红色提取 (H: 0-10 + 170-180)
  2. 膨胀 + 轮廓检测
  3. approxPolyDP 四边形逼近 (epsilon=5%)
  4. 对角线比例检查 → 确认是正方形/矩形
  5. 取最大方形, 计算中心偏移和尺寸

发布话题:
  /vision/frame_box  Float32MultiArray
    data[0] = found           (0.0 或 1.0)
    data[1] = cx_error        (-1到1, 画面中心为0)
    data[2] = cy_error        (-1到1, 画面中心为0)
    data[3] = diagonal_length (像素, 用于距离估算)

调参:
  所有关键参数均为 ROS 2 参数, 支持运行时动态调整
  ros2 param set /frame_detect_node min_area 1500
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .image_processing import LatestImageSubscription


def diagonal_ratio(points):
    """计算四边形对角线比例 (越小越接近正方形, 官方阈值<0.2)"""
    d1 = np.linalg.norm(points[0] - points[2])
    d2 = np.linalg.norm(points[1] - points[3])
    if d1 + d2 < 1e-6:
        return 1.0
    return abs(d1 - d2) / max(d1, d2)


def detect_red_square(frame, hsv_params, min_area, diag_threshold, approx_epsilon=0.05):
    """
    检测红色方框 (直接从官方 CrossRing3.py 移植)。
    参数:
      frame:           BGR 图像
      hsv_params:      (lower1, upper1, lower2, upper2) HSV 阈值
      min_area:        最小轮廓面积 (px)
      diag_threshold:  对角线比阈值
      approx_epsilon:  approxPolyDP 精度 (官方 0.05 = 5%)
    返回: (found, cx_error, cy_error, diagonal_length)
    """
    h, w = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower1, upper1, lower2, upper2 = hsv_params

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    # M7修复: 位或代替加法, 防止uint8溢出 (255+255=254 mod 256)
    mask = cv2.bitwise_or(mask1, mask2)

    # 膨胀 (连接断开的轮廓)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)

    cnts, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for cnt in cnts:
        cnt_len = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, approx_epsilon * cnt_len, True)
        if (len(approx) == 4
                and cv2.contourArea(approx) > min_area
                and cv2.isContourConvex(approx)):
            approx = approx.reshape(-1, 2)
            if diagonal_ratio(approx) < diag_threshold:
                squares.append(approx)

    if not squares:
        return False, 0.0, 0.0, 0.0

    # 取面积最大的方形
    largest = max(squares, key=cv2.contourArea)
    cx = (largest[0][0] + largest[2][0]) / 2.0
    cy = (largest[0][1] + largest[2][1]) / 2.0
    diag = np.linalg.norm(largest[0] - largest[2])

    cx_error = (cx - w / 2.0) / (w / 2.0)
    cy_error = (cy - h / 2.0) / (h / 2.0)

    return True, cx_error, cy_error, diag


class FrameDetectNode(Node):
    """红色方框检测 ROS 2 节点"""

    def __init__(self):
        super().__init__("frame_detect_node")

        # ===== 检测参数 =====
        self.declare_parameter("min_area", 2000)
        self.declare_parameter("diagonal_ratio_threshold", 0.2)
        self.declare_parameter("approx_epsilon", 0.05)
        self.declare_parameter("image_topic", "/camera/down/image_raw")

        # HSV 红色双阈值 (可动态调参)
        self.declare_parameter("hsv_lower_red1", [0, 25, 25])
        self.declare_parameter("hsv_upper_red1", [10, 255, 255])
        self.declare_parameter("hsv_lower_red2", [170, 25, 25])
        self.declare_parameter("hsv_upper_red2", [180, 255, 255])

        self._cache_params()

        self.bridge = CvBridge()

        self.frame_pub = self.create_publisher(
            Float32MultiArray, "/vision/frame_box", 10)

        self.image_pipeline = LatestImageSubscription(self, self._p('image_topic'), self.image_callback)

        self.get_logger().info(
            f"方框检测节点已启动: min_area={self.min_area}, "
            f"diag_threshold={self.diag_threshold}"
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _cache_params(self):
        self.min_area = int(self._p("min_area"))
        self.diag_threshold = float(self._p("diagonal_ratio_threshold"))
        self.approx_epsilon = float(self._p("approx_epsilon"))
        self.hsv_params = (
            np.array(self._p("hsv_lower_red1")),
            np.array(self._p("hsv_upper_red1")),
            np.array(self._p("hsv_lower_red2")),
            np.array(self._p("hsv_upper_red2")),
        )

    def image_callback(self, msg: Image):
        # 每30帧刷新参数缓存
        fc = getattr(self, "_fc", 0) + 1
        self._fc = fc
        if fc % 30 == 0:
            self._cache_params()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # M9修复: 检测函数加异常保护, 单帧畸形数据不导致节点崩溃
        try:
            found, cx_err, cy_err, diag = detect_red_square(
                frame, self.hsv_params, self.min_area,
                self.diag_threshold, self.approx_epsilon
            )
        except Exception as e:
            self.get_logger().error(f"方框检测异常: {e}")
            return

        result = Float32MultiArray()
        result.data = [
            1.0 if found else 0.0,
            float(cx_err), float(cy_err), float(diag),
        ]
        self.frame_pub.publish(result)

        if found:
            self.get_logger().debug(
                f"方框: cx={cx_err:.3f} cy={cy_err:.3f} diag={diag:.0f}px",
                throttle_duration_sec=1.0,
            )


def main():
    rclpy.init()
    node = FrameDetectNode()
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
