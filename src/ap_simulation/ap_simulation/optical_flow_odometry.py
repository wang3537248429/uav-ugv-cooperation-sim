"""Optical-flow + range 3-D odometry primitives.

The estimator intentionally has no ROS dependencies. Optical flow supplies
body-frame velocity, the range sensor supplies local vertical displacement,
and flight attitude rotates velocity into the local frame. Horizontal position
is relative and will drift without a VIO/SLAM or other position correction.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class OpticalFlowSample:
    stamp: float
    velocity_body: Vector3
    quality: int
    height_m: float


@dataclass(frozen=True)
class Odometry3D:
    stamp: float
    position_ned: Vector3
    velocity_ned: Vector3
    covariance: float


class FlowTimestampMapper:
    """Map sensor timestamps to one monotonic integration-time domain.

    A missing/zero camera timestamp falls back to its receive time and resets
    the source-clock offset. The next valid source stamp is then anchored to
    that same steady clock, so a zero epoch never causes a backward time jump.
    """

    def __init__(self):
        self._offset: Optional[float] = None
        self._last_source_stamp: Optional[float] = None

    def to_monotonic_seconds(self, source_stamp: float, receipt_time: float) -> float:
        if not math.isfinite(source_stamp) or source_stamp <= 0.0:
            self._offset = None
            self._last_source_stamp = None
            return float(receipt_time)
        if self._offset is None or (
            self._last_source_stamp is not None
            and source_stamp < self._last_source_stamp
        ):
            self._offset = float(receipt_time) - source_stamp
        self._last_source_stamp = source_stamp
        return source_stamp + self._offset


def body_flu_to_ned(vector: Vector3, roll: float, pitch: float, yaw: float) -> Vector3:
    """Rotate a ROS body-FLU vector into the local north-east-down frame."""
    # ROS base_link is FLU; MAVLink/NED body axes are FRD.
    x, y, z = vector[0], -vector[1], -vector[2]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1 = x
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    x2 = cp * x1 + sp * z1
    y2 = y1
    z2 = -sp * x1 + cp * z1
    return cy * x2 - sy * y2, sy * x2 + cy * y2, z2


def ned_to_body_flu(vector: Vector3, roll: float, pitch: float, yaw: float) -> Vector3:
    """Rotate a local NED vector into the ROS body-FLU frame."""
    north, east, down = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    x2 = cy * north + sy * east
    y1 = -sy * north + cy * east
    z2 = down
    x1 = cp * x2 - sp * z2
    z1 = sp * x2 + cp * z2
    body_forward = x1
    body_right = cr * y1 + sr * z1
    body_down = -sr * y1 + cr * z1
    return body_forward, -body_right, -body_down


def attitude_rpy_to_ned_flu_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """Convert ArduPilot FRD attitude to a NED-to-ROS-FLU pose quaternion.

    The FCU reports the rotation from body-FRD into local NED, while ROS
    ``base_link`` uses body-FLU. Compose the FCU attitude with the fixed
    180-degree FRD-to-FLU rotation about the forward axis.
    """
    half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qw, qz, -qy, -qx


def optical_flow_body_to_ned(
    velocity_body: Vector3,
    down_velocity: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> Vector3:
    """Recover 3-D NED velocity from planar flow and range-derived ``v_down``.

    A downward camera measures the forward/right components in the body plane;
    it does not measure body-down translation.  With the range derivative as
    the third NED constraint, solve for the missing body-down component before
    applying the full attitude rotation.  Treating the missing component as
    zero is only correct while the vehicle is level and stationary vertically.
    """
    forward, left, _unused_body_down = velocity_body
    body_right = -left  # FLU -> FRD
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    denominator = cp * cr
    if abs(denominator) < 1.0e-6:
        return body_flu_to_ned(velocity_body, roll, pitch, yaw)
    body_down = (
        float(down_velocity) + sp * forward - cp * sr * body_right
    ) / denominator
    return body_flu_to_ned((forward, left, -body_down), roll, pitch, yaw)


class OpticalFlowIntegrator:
    """Integrate quality-gated flow velocity into a local NED estimate."""

    def __init__(self, *, min_quality: int = 80, max_dt: float = 0.20,
                 max_speed: float = 3.0, velocity_alpha: float = 0.35):
        self.min_quality = max(0, min(255, int(min_quality)))
        self.max_dt = max(0.01, float(max_dt))
        self.max_speed = max(0.1, float(max_speed))
        self.velocity_alpha = max(0.01, min(1.0, float(velocity_alpha)))
        self._last_stamp: Optional[float] = None
        self._position: Vector3 = (0.0, 0.0, 0.0)
        self._velocity: Vector3 = (0.0, 0.0, 0.0)
        self._range_origin: Optional[float] = None
        self._last_z_down: Optional[float] = None

    @property
    def position(self) -> Vector3:
        return self._position

    @property
    def velocity(self) -> Vector3:
        return self._velocity

    def reset(self, position: Vector3 = (0.0, 0.0, 0.0), stamp: Optional[float] = None) -> None:
        self._position = tuple(float(x) for x in position)  # type: ignore[assignment]
        self._velocity = (0.0, 0.0, 0.0)
        self._last_stamp = stamp
        self._range_origin = None
        self._last_z_down = None

    def update(
        self,
        sample: OpticalFlowSample,
        attitude_rpy: Vector3 = (0.0, 0.0, 0.0),
    ) -> Optional[Odometry3D]:
        if sample.quality < self.min_quality or sample.height_m <= 0.02:
            return None
        if not all(math.isfinite(x) for x in (*sample.velocity_body, sample.height_m, sample.stamp)):
            return None
        roll, pitch, yaw = attitude_rpy
        if not all(math.isfinite(x) for x in attitude_rpy):
            return None
        vertical_range = sample.height_m * math.cos(roll) * math.cos(pitch)
        if self._range_origin is None:
            self._range_origin = vertical_range + self._position[2]
        z_down = -(vertical_range - self._range_origin)
        if self._last_stamp is None:
            self._last_stamp = sample.stamp
            self._last_z_down = z_down
            self._position = (self._position[0], self._position[1], z_down)
            return Odometry3D(sample.stamp, self._position, self._velocity, 1.0)
        dt = sample.stamp - self._last_stamp
        if dt <= 0.0:
            return None
        if dt > self.max_dt:
            self._last_stamp = sample.stamp
            self._last_z_down = z_down
            return None
        previous_z_down = self._last_z_down if self._last_z_down is not None else z_down
        down_velocity = (z_down - previous_z_down) / dt
        velocity_ned = optical_flow_body_to_ned(
            sample.velocity_body, down_velocity, roll, pitch, yaw
        )
        if math.sqrt(sum(value * value for value in velocity_ned)) > self.max_speed:
            # Consume rejected samples so one spike does not stretch the next
            # integration interval or repeatedly contaminate the range rate.
            self._last_stamp = sample.stamp
            self._last_z_down = z_down
            return None
        a = self.velocity_alpha
        self._velocity = tuple(a * new + (1.0 - a) * old for new, old in zip(velocity_ned, self._velocity))
        # A flow sample is the average body velocity over the interval ending
        # at its timestamp. Integrate that interval measurement directly;
        # applying the display/output low-pass first would introduce phase lag
        # and accumulate position error during acceleration and braking.
        self._position = tuple(
            old + measured * dt
            for old, measured in zip(self._position, velocity_ned)
        )
        self._position = (self._position[0], self._position[1], z_down)
        self._last_stamp = sample.stamp
        self._last_z_down = z_down
        covariance = max(0.01, (1.0 - sample.quality / 255.0) * 0.5)
        return Odometry3D(sample.stamp, self._position, self._velocity, covariance)
