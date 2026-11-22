"""
仿真场景定义 + 透视相机合成 (第28届空地协同规则)

坐标系 (本地NED平面 + Up高度):
  x = North/前进, y = East/右向, z = Up
  原点为UAV起飞点。Gazebo世界中UAV起点是(-2.25, 0)，
  因此Gazebo物体坐标减去起点偏移后才是这里的传感器场景坐标。

针孔相机模型:
  - HFOV=60°, fx=fy=320/tan(30°)≈554
  - 目标3D世界坐标 → 2D图像坐标透视投影
  - 无人机高度影响目标在图像中的大小和位置
"""

import math
import numpy as np

from .competition_paths import ugv_strike_pose_ned


# ========== 相机内参 (640x480, HFOV=60°) ==========
CAM_FX = 554.0  # 320 / tan(30°)
CAM_FY = 554.0
CAM_CX = 320.0
CAM_CY = 240.0

# ========== 场景定义 ==========


def get_default_scene():
    """6x5m 比赛场地: 2柱子 + 方框 + 4墙 + 气球位置(世界坐标) + ArUco地面码位置"""
    strike_x, strike_y = ugv_strike_pose_ned()
    return {
        # 方形障碍柱: 0.5x0.5m (规则1.5.2)
        "pillar1": {"x": 2.25, "y": 0.75, "width": 0.5, "depth": 0.5},
        "pillar2": {"x": 2.25, "y": -0.75, "width": 0.5, "depth": 0.5},
        # 方框立柱: 0.05x0.05m, 净孔宽约0.95m (规则要求孔径>=0.9m)
        "frame_left":  {"x": 3.75, "y": 0.5, "width": 0.05, "depth": 0.05},
        "frame_right": {"x": 3.75, "y": -0.5, "width": 0.05, "depth": 0.05},
        # 红气球世界坐标 (装在UGV顶部, 直径0.16m)
        "balloon_world": (strike_x, strike_y, 0.24),
        "balloon_radius": 0.08,
        "balloon_visible": True,
        # 地面码世界坐标 (方框后约0.25m, 贴在地面)
        "aruco_world": (4.0, 0.0, 0.0),
        "aruco_size": 0.15,  # 15cm
        "aruco_id": 0,
        "aruco_dictionary": "DICT_4X4_50",
        "aruco_visible": True,
        # 场地边界
        "walls": [
            {"x0": -0.75, "y0": -2.5, "x1": 5.25, "y1": -2.5},
            {"x0": -0.75, "y0": 2.5, "x1": 5.25, "y1": 2.5},
            {"x0": 5.25, "y0": -2.5, "x1": 5.25, "y1": 2.5},
            {"x0": -0.75, "y0": -2.5, "x1": -0.75, "y1": 2.5},
        ],
    }


# ========== 射线-矩形线段求交 ==========
def _ray_segment_intersect(ox, oy, dx, dy, ax, ay, bx, by):
    ex = bx - ax
    ey = by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-10:
        return float("inf")
    t = ((ax - ox) * ey - (ay - oy) * ex) / denom
    u = ((ax - ox) * dy - (ay - oy) * dx) / denom
    if t > 0 and 0 <= u <= 1:
        return t
    return float("inf")


def _ray_box_intersect(ox, oy, dx, dy, cx, cy, hw, hd):
    best = float("inf")
    best = min(best, _ray_segment_intersect(ox, oy, dx, dy, cx - hw, cy - hd, cx + hw, cy - hd))
    best = min(best, _ray_segment_intersect(ox, oy, dx, dy, cx + hw, cy - hd, cx + hw, cy + hd))
    best = min(best, _ray_segment_intersect(ox, oy, dx, dy, cx + hw, cy + hd, cx - hw, cy + hd))
    best = min(best, _ray_segment_intersect(ox, oy, dx, dy, cx - hw, cy + hd, cx - hw, cy - hd))
    return best


def cast_lidar_ray(origin_x, origin_y, angle, scene, max_range=12.0):
    dx = math.cos(angle)
    dy = math.sin(angle)
    best = float("inf")
    for key in ("pillar1", "pillar2"):
        p = scene[key]
        d = _ray_box_intersect(origin_x, origin_y, dx, dy, p["x"], p["y"], p["width"] / 2, p["depth"] / 2)
        best = min(best, d)
    for key in ("frame_left", "frame_right"):
        p = scene[key]
        d = _ray_box_intersect(origin_x, origin_y, dx, dy, p["x"], p["y"], p["width"] / 2, p["depth"] / 2)
        best = min(best, d)
    for w in scene["walls"]:
        d = _ray_segment_intersect(origin_x, origin_y, dx, dy, w["x0"], w["y0"], w["x1"], w["y1"])
        best = min(best, d)
    if best > max_range:
        return float("inf")
    return best


# ========== 针孔投影 ==========
def world_to_pixel(wx, wy, wz, drone_x, drone_y, drone_alt):
    """
    世界坐标 (wx,wy,wz) → 图像坐标 (px,py)
    相机在 (drone_x, drone_y, drone_alt) 处向下看
    wz=地面高度, 通常为0
    """
    # 相机坐标系: 无人机下方, 下视
    dx = wx - drone_x     # 世界X → 相机X (图像横向)
    dy = wy - drone_y     # 世界Y → 相机Y (图像纵向)
    dz = drone_alt - wz   # 高度差

    if dz < 0.05:  # 太接近地面
        dz = 0.05

    px = CAM_CX + CAM_FX * dx / dz
    py = CAM_CY + CAM_FY * dy / dz
    return px, py, dz


def _static_ground(seed=42):
    """静态地面纹理 (固定种子, 模拟真实地面)"""
    rng = np.random.RandomState(seed)
    # 预生成大纹理然后裁剪
    ground = rng.randint(45, 110, (480, 640, 3), dtype=np.uint8)
    # 添加一些较大的色块模拟地面特征
    for _ in range(12):
        cx = rng.randint(0, 640)
        cy = rng.randint(0, 480)
        radius = rng.randint(15, 40)
        color = rng.randint(40, 100, 3)
        y_min = max(0, cy - radius)
        y_max = min(480, cy + radius)
        x_min = max(0, cx - radius)
        x_max = min(640, cx + radius)
        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                if (x - cx) ** 2 + (y - cy) ** 2 < radius ** 2:
                    ground[y, x] = (ground[y, x] * 0.5 + color * 0.5).astype(np.uint8)
    return ground


def _make_aruco_marker(dictionary_name, marker_id, size_px):
    import cv2

    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        dictionary_id = cv2.aruco.DICT_4X4_50
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    generator = getattr(cv2.aruco, "generateImageMarker", None)
    if generator is not None:
        marker = generator(dictionary, int(marker_id), int(size_px))
    else:
        marker = cv2.aruco.drawMarker(dictionary, int(marker_id), int(size_px))
    if marker.ndim == 2:
        marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return marker


def _blit_marker(img, marker, cx, cy, width, height):
    mh, mw = marker.shape[:2]
    x0 = int(cx - mw / 2)
    y0 = int(cy - mh / 2)
    if x0 >= 0 and y0 >= 0 and x0 + mw < width and y0 + mh < height:
        img[y0:y0 + mh, x0:x0 + mw] = marker
        return
    clip_x0 = max(0, x0)
    clip_y0 = max(0, y0)
    clip_x1 = min(width, x0 + mw)
    clip_y1 = min(height, y0 + mh)
    if clip_x1 > clip_x0 and clip_y1 > clip_y0:
        mx0 = clip_x0 - x0
        my0 = clip_y0 - y0
        img[clip_y0:clip_y1, clip_x0:clip_x1] = marker[
            my0:my0 + (clip_y1 - clip_y0),
            mx0:mx0 + (clip_x1 - clip_x0),
        ]


_G_STATIC_GROUND = None  # 缓存


def generate_camera_image(drone_x, drone_y, drone_alt, scene=None, width=640, height=480):
    """
    针孔透视合成下视相机图像 (bgr8 numpy array).

    参数:
      drone_x, drone_y: 无人机世界坐标 (m)
      drone_alt: 无人机高度 (m)
      scene: 场景字典 (默认使用比赛场地)
      width, height: 图像尺寸
    """
    global _G_STATIC_GROUND
    import cv2

    if scene is None:
        scene = get_default_scene()

    # 静态地面纹理
    if _G_STATIC_GROUND is None:
        _G_STATIC_GROUND = _static_ground()
    img = _G_STATIC_GROUND.copy()

    # 相机外参 (下视, 光轴-Z)
    cam_x = drone_x
    cam_y = drone_y
    cam_z = drone_alt

    # --- 绘制红气球 (透视投影, 贴在地面wz≈0.1) ---
    if scene.get("balloon_visible", True):
        bw_x, bw_y, bw_z = scene["balloon_world"]
        bpx, bpy, bdz = world_to_pixel(bw_x, bw_y, bw_z, cam_x, cam_y, cam_z)
        # 气球在地面的投影半径 (透视缩放)
        radius_3d = scene["balloon_radius"]
        radius_px = int(radius_3d * CAM_FX / bdz)
        radius_px = max(5, min(200, radius_px))
        if 0 <= bpx < width and 0 <= bpy < height:
            cv2.circle(img, (int(bpx), int(bpy)), radius_px, (0, 0, 255), -1)
            cv2.circle(img, (int(bpx), int(bpy)), radius_px, (0, 0, 200), 2)

    # --- 绘制方框 (透视投影, 1.5m处, 净孔宽约0.95m) ---
    # 方框实际在 (frame_x=3.75 ~ 3.80, frame_y=-0.5 ~ 0.5)
    # 简化为4条边组成矩形开口
    frame_x0 = 3.75  # 近边
    frame_x1 = 3.80  # 远边 (框厚5cm)
    frame_y0 = -0.5
    frame_y1 = 0.5

    # 投影四个角
    corners = []
    for fx in (frame_x0, frame_x1):
        for fy in (frame_y0, frame_y1):
            px, py, dz = world_to_pixel(fx, fy, 0.0, cam_x, cam_y, cam_z)
            corners.append((int(px), int(py)))

    # 绘制方框为 fillPoly
    if all(0 <= p[0] < width and 0 <= p[1] < height for p in corners):
        # 按正确的顺序重排
        pts_ordered = np.array([
            corners[0], corners[2], corners[3], corners[1]  # 外矩形
        ], dtype=np.int32)
        cv2.polylines(img, [pts_ordered], isClosed=True, color=(0, 0, 255), thickness=3)
        # 填充半透明红色
        overlay = img.copy()
        cv2.fillPoly(overlay, [pts_ordered], (0, 0, 200))
        img = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

    # --- 地面网格纹理 (透视参考线) ---
    for gx in np.arange(-0.5, 5.5, 0.5):
        for gy in np.arange(-2.0, 2.5, 0.5):
            px, py, dz = world_to_pixel(gx, gy, 0.0, cam_x, cam_y, cam_z)
            if 0 <= px < width and 0 <= py < height:
                cv2.circle(img, (int(px), int(py)), 1, (80, 80, 80), -1)

    # Keep the marker over the synthetic grid so grid dots cannot corrupt its
    # binary cells and make detection depend on the sub-pixel camera position.
    if scene.get("aruco_visible", True):
        aruco_x, aruco_y, aruco_z = scene["aruco_world"]
        apx, apy, adz = world_to_pixel(aruco_x, aruco_y, aruco_z, cam_x, cam_y, cam_z)
        aruco_size_px = int(scene["aruco_size"] * CAM_FX / adz)
        aruco_size_px = max(24, min(220, aruco_size_px))
        if 0 <= apx < width and 0 <= apy < height and aruco_size_px > 30:
            marker = _make_aruco_marker(
                scene.get("aruco_dictionary", "DICT_4X4_50"),
                scene.get("aruco_id", 0),
                aruco_size_px,
            )
            _blit_marker(img, marker, apx, apy, width, height)

    return img
