#!/usr/bin/env python3
"""Smooth bounded actor updates using persistent ROS/Gazebo service bridges."""

import math
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from std_msgs.msg import Bool, Float32, Int32

from .competition_paths import balloon_pose_gazebo, ugv_pose_gazebo
from .random_marker import marker_sdf
from .ugv_geometry import check_pose, load_obstacles, validate_path


class GzEntitySync(Node):
    """Move scene actors only; UAV motor physics remains independent."""

    def __init__(self):
        super().__init__('gz_entity_sync')
        for key, value in [('world_name', 'competition_arena'), ('ugv_model', 'ugv_rover'),
                           ('balloon_model', 'red_balloon'), ('sync_rate', 30.)]:
            self.declare_parameter(key, value)
        world = str(self.get_parameter('world_name').value)
        self.ugv_model = str(self.get_parameter('ugv_model').value)
        self.balloon_model = str(self.get_parameter('balloon_model').value)
        share = Path(get_package_share_directory('ap_simulation'))
        self.obstacles = load_obstacles(share / 'worlds/competition_world.sdf')
        samples = validate_path(self.obstacles)
        self.pose_client = self.create_client(SetEntityPose, f'/world/{world}/set_pose')
        self.spawn_client = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.delete_client = self.create_client(DeleteEntity, f'/world/{world}/remove')
        self.applied_pub = self.create_publisher(Float32, '/simulation/ugv_pose_applied', 10)
        self.ugv_progress = 0.
        self.balloon_punctured = False
        self.pending = {}
        self.last_error = 0.
        self.marker_id = None
        self.active_marker_id = None
        self.original_marker_hidden = False
        self.marker_future = None
        self.marker_model_file = share / 'models/aruco_marker/model.sdf'
        self.create_subscription(Float32, '/mission/ugv_progress', self._ugv_cb, 1)
        self.create_subscription(Bool, '/mission/balloon_punctured', self._balloon_cb, 1)
        self.create_subscription(Int32, '/simulation/aruco_id', self._marker_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1. / max(1., float(self.get_parameter('sync_rate').value)), self._sync)
        self.get_logger().info(f'UGV route checked: {samples} swept-footprint samples; persistent async services')

    def _ugv_cb(self, message):
        self.ugv_progress = max(0., min(1., float(message.data)))

    def _balloon_cb(self, message):
        self.balloon_punctured = bool(message.data)

    def _marker_cb(self, message):
        self.marker_id = int(message.data)

    def _error(self, message):
        now = time.monotonic()
        if now - self.last_error > 5.:
            self.get_logger().warning(message)
            self.last_error = now

    def _set_pose(self, name, x, y, z, yaw, progress=None):
        if name in self.pending:
            future, sent = self.pending[name]
            if not future.done() and time.monotonic() - sent < 2.:
                return
            if not future.done():
                future.cancel()
            self.pending.pop(name, None)
        if not self.pose_client.service_is_ready():
            self._error('Waiting for Gazebo pose service bridge')
            return
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = map(float, (x, y, z))
        request.pose.orientation.z = math.sin(yaw / 2)
        request.pose.orientation.w = math.cos(yaw / 2)
        future = self.pose_client.call_async(request)
        self.pending[name] = future, time.monotonic()

        def done(result):
            if result.cancelled():
                return
            try:
                if not result.result().success:
                    self._error(f'Gazebo refused pose for {name}')
                elif progress is not None:
                    self.applied_pub.publish(Float32(data=progress))
                elif name == 'aruco_marker':
                    self.original_marker_hidden = True
            except Exception as error:
                self._error(str(error))
        future.add_done_callback(done)

    def _sync(self):
        pose = ugv_pose_gazebo(self.ugv_progress)
        try:
            check_pose(pose[0], pose[1], pose[3], self.obstacles)
        except ValueError as error:
            self._error(f'UGV motion blocked: {error}')
            return
        self._set_pose(self.ugv_model, *pose, progress=self.ugv_progress)
        x, y, z, yaw = balloon_pose_gazebo(self.ugv_progress)
        self._set_pose(self.balloon_model, x, y, -5. if self.balloon_punctured else z, yaw)
        self._sync_marker()

    def _sync_marker(self):
        if self.marker_id is None or self.marker_id == self.active_marker_id:
            return
        if self.marker_future is not None and not self.marker_future.done():
            return
        if not self.original_marker_hidden:
            self._set_pose('aruco_marker', 1.75, 0., -5., 0.)
            return
        if self.active_marker_id is not None:
            if not self.delete_client.service_is_ready():
                return
            request = DeleteEntity.Request(entity=Entity(name='round_aruco', type=Entity.MODEL))
            self.marker_future = self.delete_client.call_async(request)

            def deleted(future):
                if not future.cancelled():
                    try:
                        if future.result().success:
                            self.active_marker_id = None
                        else:
                            self._error('Gazebo refused previous ArUco removal')
                    except Exception as error:
                        self._error(f'ArUco removal failed: {error}')
            self.marker_future.add_done_callback(deleted)
            return
        if not self.spawn_client.service_is_ready():
            return
        request = SpawnEntity.Request()
        request.entity_factory.name = 'round_aruco'
        request.entity_factory.sdf = marker_sdf(
            self.marker_id, os.path.join(
                os.environ.get('COMPETITION_LOCAL_DIR', os.path.expanduser('~/.cache/aeromind-sim')),
                'marker-cache'), self.marker_model_file)
        request.entity_factory.pose.position.x = 1.75
        request.entity_factory.pose.position.z = .001
        request.entity_factory.pose.orientation.w = 1.
        selected = self.marker_id
        self.marker_future = self.spawn_client.call_async(request)

        def created(future):
            if not future.cancelled():
                try:
                    if future.result().success:
                        self.active_marker_id = selected
                        self.get_logger().info(f'Gazebo ground marker synchronized: ArUco ID={selected}')
                    else:
                        self._error('Gazebo refused round ArUco creation')
                except Exception as error:
                    self._error(f'ArUco creation failed: {error}')
        self.marker_future.add_done_callback(created)


def main():
    rclpy.init()
    node = GzEntitySync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
