"""Pure 3-D guidance primitives used by the UAV mission.

The mission uses a local NED frame for MAVLink messages, but waypoints store
altitude as a positive-up value because that is easier to review.  This module
uses ``z_up`` internally and converts to NED only at the message boundary.
Keeping the controller free of ROS objects makes it straightforward to test
with recorded optical-flow/VIO data.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _scale(value: Vector3, factor: float) -> Vector3:
    return value[0] * factor, value[1] * factor, value[2] * factor


def norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def clamp_norm(value: Vector3, maximum: float) -> Vector3:
    """Limit a vector magnitude without changing its direction."""
    magnitude = norm(value)
    if magnitude <= maximum or magnitude <= 1.0e-9:
        return value
    return _scale(value, maximum / magnitude)


@dataclass(frozen=True)
class SegmentProjection3D:
    """Projection of a point onto a finite 3-D line segment."""

    point: Vector3
    along: float
    length: float
    cross_track: Vector3


def project_to_segment(current: Vector3, start: Vector3, target: Vector3) -> SegmentProjection3D:
    """Project ``current`` onto ``start -> target`` and return 3-D error."""
    segment = _sub(target, start)
    length = norm(segment)
    if length <= 1.0e-9:
        return SegmentProjection3D(target, 0.0, 0.0, _sub(current, target))
    unit = _scale(segment, 1.0 / length)
    along = max(0.0, min(length, sum(_sub(current, start)[i] * unit[i] for i in range(3))))
    projected = _add(start, _scale(unit, along))
    return SegmentProjection3D(projected, along, length, _sub(current, projected))


def lookahead_target_3d(
    current: Vector3,
    start: Vector3,
    target: Vector3,
    lookahead: float,
) -> SegmentProjection3D:
    """Return a bounded look-ahead target on a 3-D waypoint segment."""
    projection = project_to_segment(current, start, target)
    if projection.length <= 1.0e-9:
        return projection
    unit = _scale(_sub(target, start), 1.0 / projection.length)
    along = min(projection.length, projection.along + max(0.0, lookahead))
    point = _add(start, _scale(unit, along))
    return SegmentProjection3D(point, along, projection.length, _sub(current, point))


@dataclass(frozen=True)
class Controller3DConfig:
    """Conservative velocity-controller limits for a small aircraft."""

    kp: float = 0.8
    kd: float = 0.35
    max_speed: float = 0.30
    max_vertical_speed: float = 0.15


class GuidedController3D:
    """A bounded, damped 3-D point/trajectory controller.

    ``position`` and ``velocity`` use (north, east, up).  The returned vector
    is converted to MAVLink NED by the caller, where ``vz_ned = -vz_up``.
    """

    def __init__(self, config: Controller3DConfig = Controller3DConfig()):
        self.config = config

    def velocity_command(
        self,
        position: Vector3,
        velocity: Vector3,
        target: Vector3,
    ) -> Vector3:
        error = _sub(target, position)
        command = _sub(_scale(error, self.config.kp), _scale(velocity, self.config.kd))
        horizontal = clamp_norm((command[0], command[1], 0.0), self.config.max_speed)
        vertical = max(-self.config.max_vertical_speed, min(self.config.max_vertical_speed, command[2]))
        return horizontal[0], horizontal[1], vertical


def waypoint_as_up(waypoint: Iterable[float]) -> Vector3:
    """Convert an (x north, y east, altitude up) waypoint to controller form."""
    values = tuple(float(value) for value in waypoint)
    if len(values) != 3:
        raise ValueError("a waypoint must contain x, y and altitude")
    return values[0], values[1], values[2]


def staged_return_target(anchor_xy, phase, home_xy=(0.0, 0.0)):
    """Return an axis-aligned home target within a wall-constrained corridor.

    The aircraft first moves along the east-west axis while holding its
    strike-zone northing, then corrects north-south position over the home
    point. This avoids long diagonal commands near the launch-area boundary.
    """
    anchor_x, _anchor_y = (float(value) for value in anchor_xy)
    home_x, home_y = (float(value) for value in home_xy)
    if phase == "RETURN_Y":
        return anchor_x, home_y
    if phase == "RETURN_X":
        return home_x, home_y
    raise ValueError(f"unsupported post-strike return phase: {phase}")


def staged_return_step(current_xy, anchor_xy, phase, max_step, home_xy=(0.0, 0.0)):
    """Advance one return target along exactly one horizontal axis."""
    current_x, current_y = (float(value) for value in current_xy)
    anchor_x, _anchor_y = (float(value) for value in anchor_xy)
    home_x, home_y = (float(value) for value in home_xy)
    step = float(max_step)
    if step <= 0.0:
        raise ValueError("max_step must be positive")

    if phase == "RETURN_Y":
        delta_y = home_y - current_y
        next_y = (
            home_y
            if abs(delta_y) <= step
            else current_y + math.copysign(step, delta_y)
        )
        return anchor_x, next_y
    if phase == "RETURN_X":
        delta_x = home_x - current_x
        next_x = (
            home_x
            if abs(delta_x) <= step
            else current_x + math.copysign(step, delta_x)
        )
        return next_x, home_y
    raise ValueError(f"unsupported post-strike return phase: {phase}")
