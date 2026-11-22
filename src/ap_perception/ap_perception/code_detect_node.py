#!/usr/bin/env python3
"""
统一码识别节点
==============
默认识别 ArUco 码, 并发布统一 /vision/code_* 话题。
本工程按已确认的新标准统一使用 ArUco, 识别结果格式固定为 ARUCO:<id>。
"""

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .image_processing import LatestImageSubscription


def get_aruco_dictionary(name: str):
    """解析 OpenCV ArUco 字典名, 无效时回退到 DICT_4X4_50。"""
    aruco = cv2.aruco
    default_name = "DICT_4X4_50"
    dictionary_id = getattr(aruco, name, None)
    if dictionary_id is None:
        dictionary_id = getattr(aruco, default_name)
    return aruco.getPredefinedDictionary(dictionary_id)


class CodeDetectNode(Node):
    """ArUco 码识别节点。"""

    def __init__(self):
        super().__init__("code_detect_node")

        self.declare_parameter("image_topic", "/camera/down/image_raw")
        self.declare_parameter('save_dir', os.path.join(
            os.environ.get('COMPETITION_LOCAL_DIR', os.path.expanduser('~/.cache/aeromind-sim')), 'evidence'))
        self.declare_parameter("enable_publish", True)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("target_id", 0)
        self.declare_parameter("accept_any_id", False)
        self.declare_parameter('detection_max_width', 320)
        # 合成相机图像的 ArUco 检测参数。
        self.declare_parameter("adaptive_thresh_win_min", 3)
        self.declare_parameter("adaptive_thresh_win_max", 53)   # 默认23, 调大覆盖更大码/更近距离
        self.declare_parameter("adaptive_thresh_win_step", 4)   # 默认10, 调小步进=更多阈值尝试
        self.declare_parameter("min_marker_perimeter_rate", 0.02)  # 默认0.03, 调小=可识别更小/更远的码
        self.declare_parameter("max_marker_perimeter_rate", 4.0)
        self.declare_parameter("polygonal_approx_accuracy", 0.05)  # 默认0.03, 放宽=容忍真实图像畸变
        self.declare_parameter("corner_refine_subpix", True)       # 亚像素角点精化, 提升定位精度
        self.declare_parameter("detect_inverted", True)            # 同时检测反色码, 兼容打印/光照反转

        self.image_topic = self.get_parameter("image_topic").value
        self.save_dir = os.path.expanduser(self.get_parameter("save_dir").value)
        self.enable_publish = bool(self.get_parameter("enable_publish").value)
        self.aruco_dictionary_name = self.get_parameter("aruco_dictionary").value
        self.target_id = int(self.get_parameter("target_id").value)
        self.accept_any_id = bool(self.get_parameter("accept_any_id").value)
        self.detection_max_width = max(160, int(self.get_parameter('detection_max_width').value))

        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_result = ""
        self.last_found = False
        self.detection_count = 0
        self._saved_ids = set()

        self.detector = None
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("当前 OpenCV 未包含 aruco 模块, 请安装 opencv-contrib-python 或系统 contrib 包")
        dictionary = get_aruco_dictionary(self.aruco_dictionary_name)
        params = cv2.aruco.DetectorParameters()
        # 真实相机调优: 更宽自适应阈值窗口 + 更细步进 + 放宽边长/多边形容差
        params.adaptiveThreshWinSizeMin = int(self.get_parameter("adaptive_thresh_win_min").value)
        params.adaptiveThreshWinSizeMax = int(self.get_parameter("adaptive_thresh_win_max").value)
        params.adaptiveThreshWinSizeStep = int(self.get_parameter("adaptive_thresh_win_step").value)
        params.minMarkerPerimeterRate = float(self.get_parameter("min_marker_perimeter_rate").value)
        params.maxMarkerPerimeterRate = float(self.get_parameter("max_marker_perimeter_rate").value)
        params.polygonalApproxAccuracyRate = float(self.get_parameter("polygonal_approx_accuracy").value)
        if bool(self.get_parameter("corner_refine_subpix").value):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        try:
            params.detectInvertedMarker = bool(self.get_parameter("detect_inverted").value)
        except Exception:
            pass
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        self.code_result_pub = self.create_publisher(String, "/vision/code_result", 10)
        self.code_found_pub = self.create_publisher(Bool, "/vision/code_found", 10)
        self.code_image_pub = self.create_publisher(Image, "/vision/code_image", 5)

        self.image_pipeline = LatestImageSubscription(self, self.image_topic, self.image_callback, default_fps=8.)
        self.create_subscription(Bool, "/vision/code_reset", self.reset_callback, 10)

        self.get_logger().info(
            f"ArUco码识别节点已启动: dict={self.aruco_dictionary_name} target_id={self.target_id} "
            f"image_topic={self.image_topic}"
        )

    def _select_detection(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None, None
        flat_ids = [int(v[0]) for v in ids]
        if self.accept_any_id:
            idx = 0
            return flat_ids[idx], corners[idx]
        for idx, marker_id in enumerate(flat_ids):
            if marker_id == self.target_id:
                return marker_id, corners[idx]
        return None, None

    def _save_detection_image(self, key, frame):
        if key in self._saved_ids:
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        safe_key = str(key).replace("/", "_").replace(":", "_")
        filename = os.path.join(self.save_dir, f"code_{safe_key}_{timestamp}.jpg")
        cv2.imwrite(filename, frame)
        self._saved_ids.add(key)
        self.get_logger().info(f"码识别证据图已保存: {filename}")

    def _detect_aruco(self, frame, display):
        assert self.detector is not None
        found = False
        result = ""
        scale = min(1., self.detection_max_width / frame.shape[1])
        detection_frame = frame
        if scale < 1.:
            detection_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        try:
            corners, ids, _ = self.detector.detectMarkers(detection_frame)
            if scale < 1.:
                corners = [corner / scale for corner in corners]
        except Exception as e:
            self.get_logger().error(f"ArUco检测异常: {e}")
            corners, ids = [], None

        marker_id, marker_corners = self._select_detection(corners, ids)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)

        if marker_id is not None:
            found = True
            result = f"ARUCO:{marker_id}"
            if result != self.last_result:
                self.detection_count += 1
                self.get_logger().info(f"ArUco识别成功: id={marker_id}")
            pts = marker_corners.reshape(-1, 2)
            cx = int(pts[:, 0].mean())
            cy = int(pts[:, 1].mean())
            cv2.circle(display, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(
                display, result, (max(0, cx - 45), max(20, cy - 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
            )
            self._save_detection_image(result, display)
        return found, result

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        display = frame.copy()
        found, result = self._detect_aruco(frame, display)

        self.last_found = found
        self.last_result = result if found else self.last_result

        if self.enable_publish:
            result_msg = String(data=self.last_result if self.last_found else "")
            found_msg = Bool(data=self.last_found)
            self.code_result_pub.publish(result_msg)
            self.code_found_pub.publish(found_msg)

        vis_frame = cv2.resize(display, (640, 480))
        vis_msg = self.bridge.cv2_to_imgmsg(vis_frame, encoding="bgr8")
        vis_msg.header.stamp = msg.header.stamp
        vis_msg.header.frame_id = "camera_down"
        self.code_image_pub.publish(vis_msg)

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.last_result = ""
            self.last_found = False
            self._saved_ids.clear()
            self.get_logger().info("码识别状态已重置")


def main(args=None):
    rclpy.init(args=args)
    node = CodeDetectNode()
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
