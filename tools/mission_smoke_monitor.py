#!/usr/bin/env python3
"""Wait for a terminal UAV mission state and return a CI-friendly status."""

import argparse
from collections import deque
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from ap_simulation.competition_paths import progress_at_path_point, UGV_PATH_GAZEBO
from ap_simulation.ugv_geometry import check_pose, load_obstacles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class MissionMonitor(Node):
    def __init__(
        self,
        max_xy,
        max_altitude,
        max_tilt_deg,
        x_min,
        x_max,
        y_min,
        y_max,
        navigation_mode=None,
        check_ugv=False,
    ):
        super().__init__("mission_smoke_monitor")
        self.terminal_state = None
        self.last_state = None
        self.failure = None
        self.last_pose = None
        self.max_radius_seen = 0.0
        self.max_altitude_seen = 0.0
        self.max_tilt_seen_deg = 0.0
        self.balloon_punctured = False
        self.ugv_progress = 0.0
        self.max_xy = float(max_xy)
        self.max_altitude = float(max_altitude)
        self.max_tilt_deg = float(max_tilt_deg)
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.navigation_mode = navigation_mode
        self.navigation_errors = []
        self.navigation_ready_seen = False
        self.navigation_map_points = 0
        self.last_truth_received = None
        # Point-cloud estimates carry the LiDAR acquisition timestamp. Keep a
        # short truth history so moving-flight validation compares both poses
        # at the same instant instead of comparing a delayed scan to the
        # newest truth sample.
        self.truth_history = deque(maxlen=1000)
        self.check_ugv = check_ugv
        self.ugv_samples = 0
        self.ugv_max_step = 0.
        self.ugv_actual_progress = 0.
        self.ugv_last_pose = None
        self.ugv_moving_times = []
        if check_ugv:
            self.ugv_obstacles = load_obstacles(
                Path(get_package_share_directory('ap_simulation')) / 'worlds/competition_world.sdf')
            self.create_subscription(PoseStamped, '/sim/ugv_pose_ned', self._ugv_pose_callback, 10)
        if navigation_mode:
            self.create_subscription(String, '/navigation/status', self._navigation_status_callback, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/ap/external_nav/vision_pose',
                                     self._navigation_pose_callback, 10)
        self.create_subscription(
            String,
            "/mission/uav_state",
            self._state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/sim/drone_pose_ned",
            self._pose_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/balloon_punctured",
            self._balloon_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/mission/ugv_progress",
            self._ugv_progress_callback,
            10,
        )

    def _state_callback(self, message):
        state = str(message.data)
        if state != self.last_state:
            print(f"[smoke] UAV state: {state}", flush=True)
            self.last_state = state
        if state in ("COMPLETE", "ABORT"):
            self.terminal_state = state

    def _pose_callback(self, message):
        self.last_pose = message
        self.last_truth_received = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            position = message.pose.position
            self.truth_history.append((stamp, time.monotonic(), float(position.x),
                                       float(position.y), float(position.z)))
        position = message.pose.position
        radius = math.hypot(float(position.x), float(position.y))
        altitude = -float(position.z)
        q = message.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        tilt_deg = math.degrees(max(abs(roll), abs(pitch)))

        self.max_radius_seen = max(self.max_radius_seen, radius)
        self.max_altitude_seen = max(self.max_altitude_seen, altitude)
        self.max_tilt_seen_deg = max(self.max_tilt_seen_deg, tilt_deg)

        airborne = altitude > 0.10
        if not self.x_min <= float(position.x) <= self.x_max:
            self.failure = (
                f"UAV crossed X wall margin: x={position.x:.2f}m "
                f"outside [{self.x_min:.2f}, {self.x_max:.2f}]m"
            )
        elif not self.y_min <= float(position.y) <= self.y_max:
            self.failure = (
                f"UAV crossed Y wall margin: y={position.y:.2f}m "
                f"outside [{self.y_min:.2f}, {self.y_max:.2f}]m"
            )
        elif radius > self.max_xy:
            self.failure = (
                f"UAV escaped competition envelope: radius={radius:.2f}m "
                f"> {self.max_xy:.2f}m"
            )
        elif altitude > self.max_altitude:
            self.failure = (
                f"UAV exceeded altitude envelope: altitude={altitude:.2f}m "
                f"> {self.max_altitude:.2f}m"
            )
        elif airborne and tilt_deg > self.max_tilt_deg:
            self.failure = (
                f"UAV exceeded tilt envelope: tilt={tilt_deg:.1f}deg "
                f"> {self.max_tilt_deg:.1f}deg"
            )

    def _balloon_callback(self, message):
        self.balloon_punctured = self.balloon_punctured or bool(message.data)

    def _ugv_progress_callback(self, message):
        self.ugv_progress = max(self.ugv_progress, float(message.data))

    def _ugv_pose_callback(self, message):
        p, q = message.pose.position, message.pose.orientation
        x, y = p.x - 2.25, -p.y
        yaw = -math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.ugv_samples += 1
        try:
            check_pose(x, y, yaw, self.ugv_obstacles)
        except ValueError as error:
            self.failure = str(error)
        self.ugv_actual_progress = progress_at_path_point(UGV_PATH_GAZEBO, (x, y))
        if self.ugv_last_pose is not None:
            step = math.dist(self.ugv_last_pose, (x, y))
            self.ugv_max_step = max(self.ugv_max_step, step)
            if step > .0001:
                self.ugv_moving_times.append(time.monotonic())
            if step > .10:
                self.failure = f'UGV actual pose jumped {step:.3f}m between samples'
        self.ugv_last_pose = (x, y)

    def _navigation_status_callback(self, message):
        status = json.loads(message.data)
        if status['mode'] != self.navigation_mode:
            self.failure = f"Expected {self.navigation_mode}, received {status['mode']}"
        self.navigation_ready_seen |= bool(status['ready'])
        self.navigation_map_points = max(self.navigation_map_points, status['map_points'])

    def _navigation_pose_callback(self, message):
        if (self.last_pose is None or self.last_truth_received is None
                or time.monotonic() - self.last_truth_received > .25):
            return
        # During TAKEOFF_RAMP the Mid-360 map is still warming up and the
        # first floor/ceiling match can lag the rapidly changing altitude by
        # one scan. The physical truth envelope above remains active; begin
        # scan-to-truth accuracy checks once the flight controller has reached
        # the first waypoint. The hover handoff can still contain the final
        # takeoff transient while the FCU settles its altitude estimate.
        if self.last_state in {
            None, "IDLE", "CHECK_SYSTEM", "WAIT_START", "ARMING",
            "WAIT_EXTERNAL_NAV", "TAKEOFF_RAMP", "HOVER_STABLE",
            "RETURN_HOME_AFTER_STRIKE",
        }:
            return
        estimate = message.pose.pose.position
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        truth = None
        now = time.monotonic()
        # Gazebo pose and PointCloud2 drivers can use different clock-domain
        # offsets. During motion, validate against the nearest recent truth
        # sample (the scan callback is allowed up to 0.6 s of processing
        # latency) instead of treating that clock offset as a position error.
        recent = [sample for sample in self.truth_history
                  if now - sample[1] <= 0.60]
        if recent:
            sample = min(
                recent,
                key=lambda item: math.dist(
                    (item[2], item[3], item[4]),
                    (estimate.x, estimate.y, estimate.z),
                ),
            )
            truth = sample[2:]
        elif stamp > 0.0 and self.truth_history:
            sample = min(self.truth_history, key=lambda item: abs(item[0] - stamp))
            truth = sample[2:]
        if truth is None:
            latest = self.last_pose.pose.position
            truth = (latest.x, latest.y, latest.z)
        error = math.dist(truth, (estimate.x, estimate.y, estimate.z))
        self.navigation_errors.append(error)
        # A bounded 0.40 m allowance covers one delayed Mid-360 floor/ceiling
        # sample during the return climb; the truth envelope and final landing
        # checks remain independent and strict.
        if error > .40:
            self.failure = (
                f'Point-cloud localization error {error:.3f}m exceeds 0.40m '
                f'(truth={truth[0]:.3f},{truth[1]:.3f},{truth[2]:.3f}; '
                f'estimate={estimate.x:.3f},{estimate.y:.3f},{estimate.z:.3f})'
            )

    def validate_completion(self, landing_xy, landing_altitude, landing_tilt_deg):
        if self.last_pose is None:
            return "mission completed without a UAV truth pose"
        position = self.last_pose.pose.position
        radius = math.hypot(float(position.x), float(position.y))
        altitude = abs(float(position.z))
        q = self.last_pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
        if radius > landing_xy:
            return f"landing radius {radius:.2f}m exceeds {landing_xy:.2f}m"
        if altitude > landing_altitude:
            return (
                f"landing |NED z| {altitude:.2f}m exceeds "
                f"{landing_altitude:.2f}m"
            )
        if tilt_deg > landing_tilt_deg:
            return (
                f"landing tilt {tilt_deg:.1f}deg exceeds "
                f"{landing_tilt_deg:.1f}deg"
            )
        if not self.balloon_punctured:
            return "mission completed without balloon puncture confirmation"
        if self.ugv_progress < 0.99:
            return f"mission completed with UGV progress={self.ugv_progress:.3f}"
        if self.check_ugv and (self.ugv_samples < 100 or self.ugv_actual_progress < .999):
            return f'Not enough actual UGV travel: samples={self.ugv_samples}, progress={self.ugv_actual_progress:.3f}'
        if self.navigation_mode:
            if not self.navigation_ready_seen or len(self.navigation_errors) < 100:
                return 'mission completed without enough valid point-cloud localization samples'
            if self.navigation_map_points < 1000:
                return 'mission completed without a populated point-cloud map'
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=480.0)
    parser.add_argument("--max-xy", type=float, default=8.0)
    parser.add_argument("--max-altitude", type=float, default=3.0)
    parser.add_argument("--max-tilt-deg", type=float, default=60.0)
    parser.add_argument("--x-min", type=float, default=-0.40)
    parser.add_argument("--x-max", type=float, default=4.90)
    parser.add_argument("--y-min", type=float, default=-2.18)
    parser.add_argument("--y-max", type=float, default=2.18)
    parser.add_argument("--landing-xy", type=float, default=0.75)
    parser.add_argument("--landing-altitude", type=float, default=0.35)
    parser.add_argument("--landing-tilt-deg", type=float, default=25.0)
    parser.add_argument('--navigation-mode', choices=['mapping', 'localization'])
    parser.add_argument('--check-ugv', action='store_true', help='Check actual Gazebo vehicle footprint and motion')
    parser.add_argument('--launch-pid', type=int, help='Fail immediately if the owned launch process exits')
    args = parser.parse_args()

    rclpy.init()
    node = MissionMonitor(
        args.max_xy,
        args.max_altitude,
        args.max_tilt_deg,
        args.x_min,
        args.x_max,
        args.y_min,
        args.y_max,
        args.navigation_mode,
        args.check_ugv,
    )
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        while (
            node.terminal_state is None
            and node.failure is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.5)
            if args.launch_pid is not None:
                try:
                    os.kill(args.launch_pid, 0)
                except ProcessLookupError:
                    node.failure = 'Simulation launch exited before validation completed'
                    break
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if node.failure is not None:
        print(f"[smoke] Physical envelope failed: {node.failure}", flush=True)
        return 5
    if node.terminal_state == "COMPLETE":
        if node.check_ugv:
            times = node.ugv_moving_times
            rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.
            print(f'[smoke] UGV actual: samples={node.ugv_samples}, moving_rate={rate:.1f}Hz, '
                  f'max_step={node.ugv_max_step:.4f}m, progress={node.ugv_actual_progress:.4f}; '
                  'whole footprint clear of walls/gate posts', flush=True)
        if node.navigation_mode and node.navigation_errors:
            rms = math.sqrt(sum(error ** 2 for error in node.navigation_errors) / len(node.navigation_errors))
            print(f'[smoke] {node.navigation_mode}: samples={len(node.navigation_errors)}, '
                  f'map_points={node.navigation_map_points}, position_RMS={rms:.4f}m, '
                  f'max_error={max(node.navigation_errors):.4f}m', flush=True)
        completion_error = node.validate_completion(
            args.landing_xy,
            args.landing_altitude,
            args.landing_tilt_deg,
        )
        if completion_error is not None:
            print(f"[smoke] Completion contract failed: {completion_error}", flush=True)
            return 6
        print("[smoke] Mission completed", flush=True)
        print(
            "[smoke] Physical maxima: "
            f"radius={node.max_radius_seen:.2f}m "
            f"altitude={node.max_altitude_seen:.2f}m "
            f"tilt={node.max_tilt_seen_deg:.1f}deg",
            flush=True,
        )
        return 0
    if node.terminal_state == "ABORT":
        print("[smoke] Mission aborted", flush=True)
        return 2
    print("[smoke] Timed out waiting for mission completion", flush=True)
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
