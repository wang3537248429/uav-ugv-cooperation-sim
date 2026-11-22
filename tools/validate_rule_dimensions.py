#!/usr/bin/env python3
"""Validate the repository's configured scene limits, not official rule compliance."""

from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


WS = Path(__file__).resolve().parents[1]
X500_SDF = (
    WS / "src" / "ap_simulation" / "models" / "competition_x500" / "model.sdf"
)
PX4_X500_BASE_SDF = (
    WS / "src" / "ap_simulation" / "models" / "x500_base" / "model.sdf"
)
X500_TEMPLATE = X500_SDF.with_suffix(".sdf.in")
WORLD_SDF = WS / "src" / "ap_simulation" / "worlds" / "competition_world.sdf"
ARUCO_SDF = (
    WS / "src" / "ap_simulation" / "models" / "aruco_marker" / "model.sdf"
)
MISSION_ROUTE = (
    WS / "src" / "ap_mission" / "ap_mission" / "competition_route.py"
)
SIM_PATHS = WS / "src" / "ap_simulation" / "ap_simulation" / "competition_paths.py"
UAV_GAZEBO_ORIGIN_X = -2.25
UAV_GAZEBO_ORIGIN_Y = 0.0
UAV_ROUTE_CLEARANCE_MIN = 0.30
UGV_GATE_DISTANCE_MIN = 1.50
UGV_GATE_TURN_DISTANCE_MIN_REAL_ROUTE = 1.20
_SIM_PATHS_MODULE = None

# competition_paths imports the mission route contract as an installed package.
sys.path.insert(0, str(WS / "src" / "ap_mission"))
sys.path.insert(0, str(WS / "src" / "ap_simulation"))


def floats(text: str):
    return [float(v) for v in text.split()]


def child_text(elem: ET.Element, name: str, default: str = "") -> str:
    child = elem.find(name)
    if child is None or child.text is None:
        return default
    return child.text.strip()


def pose_xy(elem: ET.Element):
    vals = floats(child_text(elem, "pose", "0 0 0 0 0 0"))
    return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]


def visual_by_name(root: ET.Element, name: str):
    for visual in root.iter("visual"):
        if visual.attrib.get("name") == name:
            return visual
    raise KeyError(name)


def model_by_name(root: ET.Element, name: str):
    for model in root.iter("model"):
        if model.attrib.get("name") == name:
            return model
    raise KeyError(name)


def link_by_name(root: ET.Element, name: str):
    for link in root.iter("link"):
        if link.attrib.get("name") == name:
            return link
    raise KeyError(name)


def cylinder_radius_length(visual: ET.Element):
    cyl = visual.find("./geometry/cylinder")
    if cyl is None:
        raise ValueError(f"{visual.attrib.get('name')} is not a cylinder visual")
    radius = float(child_text(cyl, "radius"))
    length = float(child_text(cyl, "length"))
    return radius, length


def box_size(visual: ET.Element):
    box = visual.find("./geometry/box")
    if box is None:
        raise ValueError(f"{visual.attrib.get('name')} is not a box visual")
    return floats(child_text(box, "size"))


def check(name: str, value: float, lo: float | None, hi: float | None, unit: str = "m"):
    ok = True
    if lo is not None and value < lo - 1.0e-9:
        ok = False
    if hi is not None and value > hi + 1.0e-9:
        ok = False
    status = "PASS" if ok else "FAIL"
    bounds = []
    if lo is not None:
        bounds.append(f">= {lo:.4f}{unit}")
    if hi is not None:
        bounds.append(f"<= {hi:.4f}{unit}")
    print(f"{status}: {name}: {value:.4f}{unit} ({', '.join(bounds)})")
    return ok


def check_x500():
    root = ET.parse(X500_SDF).getroot()
    template_root = ET.parse(X500_TEMPLATE).getroot()
    px4_include = next(
        include
        for include in template_root.iter("include")
        if child_text(include, "uri") == "model://x500_base"
    )
    scale_xyz = floats(child_text(px4_include, "scale"))
    uniform_scale = max(scale_xyz) - min(scale_xyz) <= 1.0e-9

    motors = []
    for i in range(4):
        x, y, *_ = pose_xy(link_by_name(root, f"rotor_{i}"))
        motors.append((x, y))
    motor_span = max(
        math.hypot(x0 - x1, y0 - y1)
        for x0, y0 in motors
        for x1, y1 in motors
    )

    guard_extents_x = []
    guard_extents_y = []
    for i in range(4):
        visual = visual_by_name(root, f"prop_guard_{i}")
        x, y, *_ = pose_xy(visual)
        radius, _ = cylinder_radius_length(visual)
        guard_extents_x.extend([x - radius, x + radius])
        guard_extents_y.extend([y - radius, y + radius])
    guard_x = max(guard_extents_x) - min(guard_extents_x)
    guard_y = max(guard_extents_y) - min(guard_extents_y)

    needle = visual_by_name(root, "competition_balloon_needle")
    tip = visual_by_name(root, "competition_balloon_needle_tip")
    needle_radius, needle_len = cylinder_radius_length(needle)
    tip_radius, tip_len = cylinder_radius_length(tip)
    needle_total = needle_len + tip_len
    needle_diameter = max(needle_radius, tip_radius) * 2.0

    ok = True
    ok &= check(
        "PX4 x500 uniform competition scale",
        1.0 if uniform_scale else 0.0,
        1.0,
        1.0,
        "",
    )
    ok &= check("UAV motor diagonal span", motor_span, 0.180, 0.330)
    ok &= check("UAV visual guard width X", guard_x, None, 0.330)
    ok &= check("UAV visual guard width Y", guard_y, None, 0.330)
    ok &= check("Needle exposed length", needle_total, None, 0.200)
    ok &= check("Needle diameter", needle_diameter, None, 0.003)
    return ok


def check_world():
    root = ET.parse(WORLD_SDF).getroot()
    world_ok = check_world_layout(root)
    ugv = model_by_name(root, "ugv_rover")
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]

    def include_box(visual: ET.Element):
        x, y, z, *_ = pose_xy(visual)
        sx, sy, sz = box_size(visual)
        half = [sx / 2.0, sy / 2.0, sz / 2.0]
        center = [x, y, z]
        for axis in range(3):
            mins[axis] = min(mins[axis], center[axis] - half[axis])
            maxs[axis] = max(maxs[axis], center[axis] + half[axis])

    def include_wheel(visual: ET.Element):
        x, y, z, roll, pitch, _ = pose_xy(visual)
        radius, length = cylinder_radius_length(visual)
        if abs(abs(roll) - math.pi / 2.0) < 0.05 and abs(pitch) < 0.05:
            half = [radius, length / 2.0, radius]
        else:
            half = [radius, radius, length / 2.0]
        center = [x, y, z]
        for axis in range(3):
            mins[axis] = min(mins[axis], center[axis] - half[axis])
            maxs[axis] = max(maxs[axis], center[axis] + half[axis])

    for name in (
        "chassis_visual",
        "top_plate_visual",
        "front_bumper_visual",
        "front_camera_visual",
        "front_light_l",
        "front_light_r",
        "top_forward_arrow_shaft",
        "top_forward_arrow_head",
        "rear_marker_visual",
    ):
        include_box(visual_by_name(ugv, name))
    for name in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        include_wheel(visual_by_name(ugv, name))

    ugv_len = maxs[0] - mins[0]
    ugv_width = maxs[1] - mins[1]
    ugv_height = maxs[2] - mins[2]

    balloon = model_by_name(root, "red_balloon")
    balloon_visual = visual_by_name(balloon, "balloon_visual")
    sphere = balloon_visual.find("./geometry/sphere")
    balloon_radius = float(child_text(sphere, "radius"))

    ok = world_ok
    ok &= check("UGV length", ugv_len, 0.280, 0.420)
    ok &= check("UGV width", ugv_width, 0.220, 0.380)
    ok &= check("UGV height", ugv_height, 0.150, 0.260)
    ok &= check("Balloon diameter", balloon_radius * 2.0, None, 0.300)
    return ok


def check_world_layout(root: ET.Element):
    ok = True

    south = visual_by_name(model_by_name(root, "south_wall"), "south_wall_visual")
    east = visual_by_name(model_by_name(root, "east_wall"), "east_wall_visual")
    south_size = box_size(south)
    east_size = box_size(east)
    ok &= check("Arena length", south_size[0], 6.0, 6.0)
    ok &= check("Arena width", east_size[1], 5.0, 5.0)
    ok &= check("Arena wall height", south_size[2], 3.0, 3.0)

    for name in ("pillar_1", "pillar_02"):
        pillar = model_by_name(root, name)
        visual = visual_by_name(pillar, f"{name}_visual")
        sx, sy, sz = box_size(visual)
        ok &= check(f"{name} length", sx, 0.5, None)
        ok &= check(f"{name} width", sy, 0.5, None)
        ok &= check(f"{name} height", sz, 1.5, None)

    gate = model_by_name(root, "aerial_gate")
    left_pose = pose_xy(link_by_name(gate, "gate_pillar_l"))
    right_pose = pose_xy(link_by_name(gate, "gate_pillar_r"))
    pillar_size = box_size(visual_by_name(gate, "gate_pillar_l_visual"))
    clear_width = abs(left_pose[1] - right_pose[1]) - pillar_size[1]
    bottom_pose = pose_xy(link_by_name(gate, "gate_bottom_bar"))
    bottom_size = box_size(visual_by_name(gate, "gate_bottom_bar_visual"))
    top_pose = pose_xy(link_by_name(gate, "gate_top_bar"))
    top_size = box_size(visual_by_name(gate, "gate_top_bar_visual"))
    bottom_edge_height = bottom_pose[2] - bottom_size[2] / 2.0
    clear_height = (top_pose[2] - top_size[2] / 2.0) - (bottom_pose[2] + bottom_size[2] / 2.0)
    ok &= check("Aerial gate clear width", clear_width, 0.9, None)
    ok &= check("Aerial gate bottom edge height", bottom_edge_height, 0.7, None)
    ok &= check("Aerial gate clear height", clear_height, 0.9, None)

    aruco_root = ET.parse(ARUCO_SDF).getroot()
    aruco_visual = visual_by_name(aruco_root, "visual")
    plane = aruco_visual.find("./geometry/plane")
    aruco_size = floats(child_text(plane, "size"))
    ok &= check("ArUco marker width", aruco_size[0], 0.15, None)
    ok &= check("ArUco marker height", aruco_size[1], 0.15, None)

    start_zone = model_by_name(root, "start_zone")
    start_x, start_y, *_ = pose_xy(start_zone)
    start_size = box_size(visual_by_name(start_zone, "start_zone_visual"))
    zone_min_x = start_x - start_size[0] / 2.0
    zone_max_x = start_x + start_size[0] / 2.0
    zone_min_y = start_y - start_size[1] / 2.0
    zone_max_y = start_y + start_size[1] / 2.0
    ugv_x, ugv_y, *_ = pose_xy(model_by_name(root, "ugv_rover"))
    balloon_x, balloon_y, *_ = pose_xy(model_by_name(root, "red_balloon"))
    ugv_inside = zone_min_x <= ugv_x <= zone_max_x and zone_min_y <= ugv_y <= zone_max_y
    balloon_inside = zone_min_x <= balloon_x <= zone_max_x and zone_min_y <= balloon_y <= zone_max_y
    ok &= check("UGV initial strike-zone membership", 1.0 if ugv_inside else 0.0, 1.0, 1.0, "")
    ok &= check("Balloon initial strike-zone membership", 1.0 if balloon_inside else 0.0, 1.0, 1.0, "")
    ok &= check_ugv_gates(root, (zone_min_x, zone_max_x, zone_min_y, zone_max_y))
    ok &= check_uav_route_clearance(root)
    return ok


def read_constant(name: str):
    global _SIM_PATHS_MODULE
    if _SIM_PATHS_MODULE is None:
        spec = importlib.util.spec_from_file_location("competition_paths_for_validation", SIM_PATHS)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        _SIM_PATHS_MODULE = module
    return getattr(_SIM_PATHS_MODULE, name)


def read_mission_waypoints():
    spec = importlib.util.spec_from_file_location(
        "competition_route_for_validation", MISSION_ROUTE
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.COMPETITION_WAYPOINTS_NED


def path_turn_distances(point, path):
    px, py = point
    # The first and last points are the strike zone, not UGV road turn corners.
    turns = path[1:-1]
    return [math.hypot(px - x, py - y) for x, y in turns]


def polygon_signed_area(points):
    area = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        area += x0 * y1 - x1 * y0
    return area / 2.0


def check_ugv_gates(root: ET.Element, start_rect):
    ok = True
    path = read_constant("UGV_PATH_GAZEBO")
    gate_sequence = read_constant("UGV_GATE_SEQUENCE")

    from ap_simulation.ugv_geometry import load_obstacles, validate_path
    try:
        samples = validate_path(load_obstacles(WORLD_SDF), points=path)
        print(f'PASS: UGV swept footprint and corner rotations: {samples} samples, margin=0.04m')
    except ValueError as error:
        print(f'FAIL: {error}')
        ok = False

    fx, fy = path[-1]
    final_after_gate = -2.35 <= fx <= -2.05 and -2.35 <= fy <= -1.85
    ok &= check("UGV final stop after gate3", 1.0 if final_after_gate else 0.0, 1.0, 1.0, "")

    strike_x = read_constant("UGV_STRIKE_GAZEBO_X")
    strike_y = read_constant("UGV_STRIKE_GAZEBO_Y")
    final_is_strike = math.hypot(fx - strike_x, fy - strike_y) <= 1.0e-9
    ok &= check("UGV final stop equals strike point", 1.0 if final_is_strike else 0.0, 1.0, 1.0, "")

    for idx, (label, gx, gy, model_name) in enumerate(gate_sequence, start=1):
        model_idx = model_name.rsplit("_", 1)[-1]
        gate = model_by_name(root, model_name)
        wx, wy, wz, *_ = pose_xy(gate)
        ok &= check(f"UGV {label} center X", abs(wx - gx), None, 0.02)
        ok &= check(f"UGV {label} center Y", abs(wy - gy), None, 0.02)

        left_pose = pose_xy(link_by_name(gate, f"ugv_gate_{model_idx}_left"))
        right_pose = pose_xy(link_by_name(gate, f"ugv_gate_{model_idx}_right"))
        pillar_size = box_size(visual_by_name(gate, f"ugv_g{model_idx}_l_y1"))
        clear_width = abs(left_pose[1] - right_pose[1]) - pillar_size[1]
        top_pose = pose_xy(link_by_name(gate, f"ugv_gate_{model_idx}_top"))
        top_size = box_size(visual_by_name(gate, f"ugv_gate_{model_idx}_top_visual"))
        clear_height = wz + top_pose[2] - top_size[2] / 2.0
        ok &= check(f"UGV {label} clear width", clear_width, 0.5, None)
        ok &= check(f"UGV {label} clear height", clear_height, 0.8, None)

        start_dist = math.hypot(wx - path[0][0], wy - path[0][1])
        turn_dist = min(path_turn_distances((wx, wy), path))
        turn_min = (
            UGV_GATE_TURN_DISTANCE_MIN_REAL_ROUTE
            if len(path) > 4
            else UGV_GATE_DISTANCE_MIN
        )
        ok &= check(f"UGV {label} distance from UGV start", start_dist, UGV_GATE_DISTANCE_MIN, None)
        ok &= check(f"UGV {label} distance from road turn", turn_dist, turn_min, None)
    return ok


def gazebo_xy_to_local_ned(x_gz: float, y_gz: float):
    return x_gz - UAV_GAZEBO_ORIGIN_X, -(y_gz - UAV_GAZEBO_ORIGIN_Y)


def point_rect_distance(px, py, xmin, xmax, ymin, ymax):
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


def segment_rect_distance(x0, y0, x1, y1, rect):
    xmin, xmax, ymin, ymax = rect
    seg_len = math.hypot(x1 - x0, y1 - y0)
    samples = max(2, int(math.ceil(seg_len / 0.01)) + 1)
    return min(
        point_rect_distance(
            x0 + (x1 - x0) * i / (samples - 1),
            y0 + (y1 - y0) * i / (samples - 1),
            xmin,
            xmax,
            ymin,
            ymax,
        )
        for i in range(samples)
    )


def pillar_rect_ned(root: ET.Element, name: str):
    pillar = model_by_name(root, name)
    x_gz, y_gz, *_ = pose_xy(pillar)
    visual = visual_by_name(pillar, f"{name}_visual")
    sx, sy, _ = box_size(visual)
    x_ned, y_ned = gazebo_xy_to_local_ned(x_gz, y_gz)
    return (
        x_ned - sx / 2.0,
        x_ned + sx / 2.0,
        y_ned - sy / 2.0,
        y_ned + sy / 2.0,
    )


def check_uav_route_clearance(root: ET.Element):
    waypoints = read_mission_waypoints()
    reference = read_constant("UAV_REFERENCE_WAYPOINTS_NED")
    route_match = (
        len(waypoints) == len(reference)
        and all(
            label == ref_label
            and abs(float(x) - float(ref_x)) <= 1.0e-6
            and abs(float(y) - float(ref_y)) <= 1.0e-6
            for (x, y, _z, label), (ref_x, ref_y, _ref_z, ref_label)
            in zip(waypoints, reference)
        )
    )
    ok = check("UAV mission route matches RViz reference", 1.0 if route_match else 0.0, 1.0, 1.0, "")
    strike_ned = gazebo_xy_to_local_ned(
        read_constant("UGV_STRIKE_GAZEBO_X"),
        read_constant("UGV_STRIKE_GAZEBO_Y"),
    )
    uav_strike = next(
        (float(x), float(y))
        for x, y, _z, label in waypoints
        if label == "STRIKE_ZONE"
    )
    strike_match = math.hypot(uav_strike[0] - strike_ned[0], uav_strike[1] - strike_ned[1]) <= 1.0e-6
    ok &= check("UAV STRIKE_ZONE equals UGV final stop", 1.0 if strike_match else 0.0, 1.0, 1.0, "")
    transit_points = [(float(x), float(y), label) for x, y, z, label in waypoints if float(z) > 0.0]
    pillar_rects = [
        pillar_rect_ned(root, "pillar_1"),
        pillar_rect_ned(root, "pillar_02"),
    ]
    min_clearance = float("inf")
    min_label = ""
    for (x0, y0, label0), (x1, y1, label1) in zip(transit_points, transit_points[1:]):
        for rect in pillar_rects:
            clearance = segment_rect_distance(x0, y0, x1, y1, rect)
            if clearance < min_clearance:
                min_clearance = clearance
                min_label = f"{label0}->{label1}"
    ok &= check(
        f"UAV route pillar clearance ({min_label})",
        min_clearance,
        UAV_ROUTE_CLEARANCE_MIN,
        None,
    )
    return ok


def main():
    ok = check_x500()
    ok &= check_world()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
