"""Load editable map-frame routes and convert to the mission's NED+alt contract."""

import math
from pathlib import Path

import yaml


MISSION_LABELS = ('TAKEOFF', 'APPROACH_FRAME', 'SCAN_ZONE', 'RETURN_TAKEOFF_ZONE',
                  'UGV_STOP_APPROACH', 'STRIKE_ZONE', 'BALLOON_ALT', 'LAND')


def load_route(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('frame_id') != 'uav_map':
        raise ValueError('route frame_id must be uav_map (X forward, Y left, Z up)')
    waypoints = data.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < len(MISSION_LABELS):
        raise ValueError('route is missing competition task waypoints')
    result = []
    labels = []
    last_task = None
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            raise ValueError('each waypoint must be a mapping')
        xyz = waypoint.get('position', [])
        label = waypoint.get('label', '')
        if (not isinstance(xyz, list) or len(xyz) != 3
                or not all(type(v) in (int, float) and math.isfinite(v) for v in xyz)):
            raise ValueError('each waypoint position requires three finite metre values')
        if not isinstance(label, str) or not label or label in labels or xyz[2] < 0:
            raise ValueError('waypoints need unique labels and nonnegative altitude')
        if label in MISSION_LABELS:
            last_task = label
        elif last_task == 'BALLOON_ALT':
            raise ValueError('post-balloon return uses a fixed corridor; intermediate waypoints are unsupported')
        x, y, altitude = map(float, xyz)
        result.append((x, -y, altitude, label))
        labels.append(label)
    if tuple(label for label in labels if label in MISSION_LABELS) != MISSION_LABELS:
        raise ValueError('competition task labels must occur once in the required order')
    if labels[0] != 'TAKEOFF' or labels[-1] != 'LAND':
        raise ValueError('route must start with TAKEOFF and finish with LAND')
    if result[0][2] <= 0. or result[-1][2] != 0.:
        raise ValueError('TAKEOFF altitude must be positive and LAND altitude must be zero')
    if any(abs(value) > 1e-9 for item in (result[0], result[-1]) for value in item[:2]):
        raise ValueError('this mission requires TAKEOFF and LAND at the map origin')
    return tuple(result)
