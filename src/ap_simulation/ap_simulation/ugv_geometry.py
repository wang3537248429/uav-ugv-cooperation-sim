"""Validate the whole UGV footprint against collision boxes in the actual SDF."""

import math
import xml.etree.ElementTree as ET

from .competition_paths import UGV_PATH_GAZEBO


HALF_LENGTH = .21
HALF_WIDTH = .17


def rectangle(x, y, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((-half_length, -half_width), (half_length, -half_width),
                           (half_length, half_width), (-half_length, half_width))]


def overlap(first, second):
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (b[1] - a[1], a[0] - b[0])
            left = [p[0] * axis[0] + p[1] * axis[1] for p in first]
            right = [p[0] * axis[0] + p[1] * axis[1] for p in second]
            if max(left) < min(right) or max(right) < min(left):
                return False
    return True


def _compose(parent, element):
    values = [float(value) for value in element.findtext('pose', '0 0 0 0 0 0').split()]
    x, y, z, _roll, _pitch, yaw = values
    c, s = math.cos(parent[3]), math.sin(parent[3])
    return parent[0] + c * x - s * y, parent[1] + s * x + c * y, parent[2] + z, parent[3] + yaw


def load_obstacles(world_file):
    obstacles = []
    root = ET.parse(world_file).getroot()
    for model in root.findall('./world/model'):
        if model.get('name') in ('ugv_rover', 'red_balloon'):
            continue
        model_pose = _compose((0., 0., 0., 0.), model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, link)
            for collision in link.findall('collision'):
                size = collision.findtext('./geometry/box/size')
                if size is None:
                    continue
                sx, sy, sz = map(float, size.split())
                x, y, z, yaw = _compose(link_pose, collision)
                if z + sz / 2 < .02 or z - sz / 2 > .23:
                    continue
                obstacles.append((f"{model.get('name')}/{collision.get('name')}",
                                  rectangle(x, y, yaw, sx / 2, sy / 2)))
    if not obstacles:
        raise ValueError('UGV route check found no collision geometry in the world')
    return obstacles


def check_pose(x, y, yaw, obstacles, margin=.04):
    footprint = rectangle(x, y, yaw, HALF_LENGTH + margin, HALF_WIDTH + margin)
    for name, polygon in obstacles:
        if overlap(footprint, polygon):
            raise ValueError(f'UGV footprint intersects {name} at x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')


def validate_path(obstacles, points=UGV_PATH_GAZEBO, sample_step=.02, margin=.04):
    """Sample translation and full corner rotations, not only waypoint centres."""
    samples = 0
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])]
    for a, b, yaw in zip(points, points[1:], headings):
        count = max(1, math.ceil(math.dist(a, b) / sample_step))
        for index in range(count + 1):
            ratio = index / count
            check_pose(a[0] + (b[0] - a[0]) * ratio, a[1] + (b[1] - a[1]) * ratio,
                       yaw, obstacles, margin)
            samples += 1
    for index, point in enumerate(points):
        start = headings[index - 1] if index else 0.
        finish = headings[index] if index < len(headings) else headings[-1]
        delta = math.atan2(math.sin(finish - start), math.cos(finish - start))
        count = max(1, math.ceil(abs(delta) / math.radians(3)))
        for step in range(count + 1):
            check_pose(*point, start + delta * step / count, obstacles, margin)
            samples += 1
    return samples
