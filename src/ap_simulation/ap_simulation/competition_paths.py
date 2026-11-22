"""Shared competition field helpers for Gazebo, synthetic sensors and RViz."""

import math

from ap_mission.competition_route import COMPETITION_WAYPOINTS_NED


UAV_GAZEBO_ORIGIN_X = -2.25
UAV_GAZEBO_ORIGIN_Y = 0.0
UAV_GAZEBO_ORIGIN_Z = 0.24

UGV_GAZEBO_Z = 0.02
BALLOON_GAZEBO_Z = 0.24
BALLOON_NED_Z_UP = 0.24
UGV_START_GAZEBO_X = -2.25
UGV_START_GAZEBO_Y = 1.8
UGV_STRIKE_GAZEBO_X = -2.25
UGV_STRIKE_GAZEBO_Y = -1.85
UGV_START_YAW_GAZEBO = 0.0  # +X/蓝色车头朝向门洞与车道入口

# Gazebo GUI uses x=right, y=up. The locked visual standard is:
# left/north start -> north lane -> east lane -> south lane -> final gate stop.
# The balloon strike point is the UGV final stop after the third gate.
UGV_ROUTE_WAYPOINTS = [
    (UGV_START_GAZEBO_X, UGV_START_GAZEBO_Y),
    (-1.85, 2.00),
    (2.50, 2.00),
    (2.50, -2.00),
    (-1.85, -2.00),
    (UGV_STRIKE_GAZEBO_X, UGV_STRIKE_GAZEBO_Y),
]


def _rounded_path(vertices, cut=.25):
    """Round bends inside the lane instead of instantly rotating the vehicle."""
    points = [vertices[0]]
    for previous, corner, following in zip(vertices, vertices[1:], vertices[2:]):
        before, after = math.dist(previous, corner), math.dist(corner, following)
        distance = min(cut, before * .3, after * .3)
        entry = tuple(corner[i] + (previous[i] - corner[i]) * distance / before for i in range(2))
        exit_point = tuple(corner[i] + (following[i] - corner[i]) * distance / after for i in range(2))
        points.append(entry)
        for step in range(1, 17):
            t = step / 16
            points.append(tuple((1 - t) ** 2 * entry[i] + 2 * t * (1 - t) * corner[i]
                                + t ** 2 * exit_point[i] for i in range(2)))
    points.append(vertices[-1])
    return points


UGV_PATH_GAZEBO = _rounded_path(UGV_ROUTE_WAYPOINTS)

if UGV_PATH_GAZEBO[-1] != (UGV_STRIKE_GAZEBO_X, UGV_STRIKE_GAZEBO_Y):
    raise ValueError("UGV final stop must be the balloon strike point")

UGV_GATE_SEQUENCE = [
    ("GATE1_NORTH", -0.50, 2.00, "ugv_gate_3"),
    ("GATE2_EAST", 2.50, 0.50, "ugv_gate_2"),
    ("GATE3_SOUTH", -0.50, -2.00, "ugv_gate_1"),
]

UGV_GATE_LABELS = tuple(item[0] for item in UGV_GATE_SEQUENCE)
UGV_GATE_GAZEBO = tuple((item[1], item[2]) for item in UGV_GATE_SEQUENCE)


UAV_REFERENCE_WAYPOINTS_NED = COMPETITION_WAYPOINTS_NED


def _clamp_progress(progress: float) -> float:
    return max(0.0, min(1.0, float(progress)))


def interpolate_path(points, progress: float):
    """Return x, y, yaw along a polyline at normalized progress 0..1."""
    progress = _clamp_progress(progress)
    if progress <= 0.0:
        x0, y0 = points[0]
        return x0, y0, UGV_START_YAW_GAZEBO
    if progress >= 1.0:
        x0, y0 = points[-2]
        x1, y1 = points[-1]
        return x1, y1, math.atan2(y1 - y0, x1 - x0)

    total = 0.0
    lengths = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        lengths.append(seg_len)
        total += seg_len

    target = progress * total
    walked = 0.0
    for i, seg_len in enumerate(lengths):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        if walked + seg_len >= target:
            ratio = (target - walked) / max(seg_len, 1.0e-6)
            x = x0 + (x1 - x0) * ratio
            y = y0 + (y1 - y0) * ratio
            yaw = math.atan2(y1 - y0, x1 - x0)
            return x, y, yaw
        walked += seg_len

    x0, y0 = points[-2]
    x1, y1 = points[-1]
    return x1, y1, math.atan2(y1 - y0, x1 - x0)


def polyline_length(points) -> float:
    """Return total length of a 2D polyline."""
    total = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def progress_at_path_point(points, point) -> float:
    """Return normalized progress for a point lying on, or nearest to, a path."""
    px, py = point
    total_len = polyline_length(points)
    if total_len <= 1.0e-6:
        return 0.0

    walked = 0.0
    best_progress = 0.0
    best_dist = float("inf")
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        seg_len2 = dx * dx + dy * dy
        seg_len = math.sqrt(seg_len2)
        if seg_len <= 1.0e-6:
            continue
        ratio = ((px - x0) * dx + (py - y0) * dy) / seg_len2
        ratio = max(0.0, min(1.0, ratio))
        cx = x0 + dx * ratio
        cy = y0 + dy * ratio
        dist = math.hypot(px - cx, py - cy)
        if dist < best_dist:
            best_dist = dist
            best_progress = (walked + seg_len * ratio) / total_len
        walked += seg_len
    return _clamp_progress(best_progress)


UGV_GATE_PROGRESS = tuple(
    progress_at_path_point(UGV_PATH_GAZEBO, gate) for gate in UGV_GATE_GAZEBO
)


def ugv_pose_gazebo(progress: float):
    """Return UGV pose in Gazebo world coordinates: x, y, z, yaw."""
    x, y, yaw = interpolate_path(UGV_PATH_GAZEBO, progress)
    return x, y, UGV_GAZEBO_Z, yaw


def balloon_pose_gazebo(progress: float):
    """Return balloon pose in Gazebo world coordinates, mounted on the UGV."""
    x, y, _ = interpolate_path(UGV_PATH_GAZEBO, progress)
    return x, y, BALLOON_GAZEBO_Z, 0.0


def gazebo_xy_to_local_ned(x_gz: float, y_gz: float):
    """Convert Gazebo world XY to local NED XY using the UAV takeoff origin."""
    return x_gz - UAV_GAZEBO_ORIGIN_X, -(y_gz - UAV_GAZEBO_ORIGIN_Y)


def gazebo_xy_to_rviz_map(x_gz: float, y_gz: float):
    """Convert Gazebo world XY to RViz map XY.

    RViz map origin is the UAV takeoff point. This makes RViz top view match
    Gazebo top view after a constant translation.
    """
    return x_gz - UAV_GAZEBO_ORIGIN_X, y_gz - UAV_GAZEBO_ORIGIN_Y


def local_ned_to_rviz_map(x_ned: float, y_ned: float):
    """Convert local NED XY to the RViz map view used by mission markers."""
    return x_ned, -y_ned


def ugv_strike_pose_ned():
    """Return UGV strike-zone center in local NED XY."""
    return gazebo_xy_to_local_ned(UGV_STRIKE_GAZEBO_X, UGV_STRIKE_GAZEBO_Y)


def balloon_pose_ned_scene(progress: float):
    """Return synthetic camera scene balloon position: x_north, y_east, z_up."""
    x_gz, y_gz, _, _ = balloon_pose_gazebo(progress)
    x_ned, y_ned = gazebo_xy_to_local_ned(x_gz, y_gz)
    return x_ned, y_ned, BALLOON_NED_Z_UP
