"""Live 3D mapping/localization and map-FLU -> ArduPilot-NED adapter.

Only PointCloud2 observations enter the estimator. No simulator pose, FCU
position, prebuilt scene geometry or route coordinate enters registration.
"""

import json
import os
from collections import deque
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .map_io import load_map, save_map
from .frames import map_flu_to_ned, navigation_covariance
from .pointcloud import make_cloud, read_xyz
from .registration import PointCloudMapper, transform_points


class PointCloudNavigation(Node):
    def __init__(self):
        super().__init__('pointcloud_navigation')
        defaults = {
            'mode': 'mapping', 'map_file': os.path.join(
                os.environ.get('COMPETITION_LOCAL_DIR', os.path.expanduser('~/.cache/aeromind-sim')),
                'maps', 'arena.npz'),
            'cloud_topic': '/navigation/lidar/points', 'map_frame': 'uav_map',
            'sensor_frame': 'drone/base_link/navigation_lidar',
            'sensor_xyz_rpy': [0., 0., 0.15, 0., 0., 0.],
            'initial_pose': [0.] * 6, 'voxel_size': 0.06,
            'min_range': 0.35, 'max_range': 12.,
            'max_distance': 0.35, 'max_rmse': 0.055, 'min_fitness': 0.55,
            'max_translation': 0.55, 'max_rotation': 0.50,
            'max_vertical_translation': 0.80,
            'max_vertical_step': 0.20,
            'upper_structure_quantile': 0.0,
            'z_correction_gain': 0.85,
            'z_correction_limit': 0.50,
            'stale_timeout': 1.2,
            'max_scan_points': 2400,
            'floor_z_gain': 0.85,
            'autosave_period': 15., 'path_length': 2000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(key):
            return self.get_parameter(key).value

        self.mode = str(p('mode'))
        self.map_file = FilePath(str(p('map_file'))).expanduser()
        self.map_frame = str(p('map_frame'))
        self.sensor_frame = str(p('sensor_frame'))
        self.sensor_pose = self._pose_from_parameter(p('sensor_xyz_rpy'))
        points = None
        voxel_size = float(p('voxel_size'))
        if self.mode == 'localization':
            points, voxel_size = load_map(self.map_file, self.map_frame)
        elif self.mode == 'mapping' and self.map_file.exists():
            raise ValueError(f'Mapping refuses to overwrite {self.map_file}; choose a new map_file')
        self.mapper = PointCloudMapper(
            mode=self.mode, points=points, initial_pose=p('initial_pose'), voxel_size=voxel_size,
            **{key: float(p(key)) for key in (
                'max_distance', 'max_rmse', 'min_fitness', 'max_translation', 'max_rotation',
                'max_vertical_translation', 'max_vertical_step',
                'upper_structure_quantile',
                'z_correction_gain', 'z_correction_limit',
                'max_scan_points'
            )}
        )
        self.min_range = float(p('min_range'))
        self.max_range = float(p('max_range'))
        self.stale_timeout = float(p('stale_timeout'))
        self.path_length = int(p('path_length'))
        self.last_input = None
        self.last_accepted = None
        self.last_stamp = None
        self.good_scans = 0
        self.reason = 'waiting_for_cloud'
        self.rmse = None
        self.fitness = 0.
        self.path = Path()
        self._pose_samples = deque(maxlen=3)
        self._published_pose = None
        self.path.header.frame_id = self.map_frame
        self.ready_pub = self.create_publisher(Bool, '/ap/external_nav/ready', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ap/external_nav/vision_pose', 10)
        self.cov_pub = self.create_publisher(Float32, '/ap/external_nav/covariance', 10)
        self.nav_pose_pub = self.create_publisher(PoseStamped, '/navigation/pose', 10)
        self.path_pub = self.create_publisher(Path, '/navigation/path', 2)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, '/navigation/map', durable)
        self.registered_pub = self.create_publisher(PointCloud2, '/navigation/cloud_registered', 2)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, str(p('cloud_topic')), self.cloud_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '/navigation/save_map', self.save_cb)
        self.create_timer(0.1, self.health)
        self.create_timer(2., self.publish_map)
        self.create_timer(max(1., float(p('autosave_period'))), self.autosave)
        self.get_logger().info(f'3D navigation mode={self.mode}, map={self.map_file}; input=PointCloud2 only')

    @staticmethod
    def _pose_from_parameter(values):
        from .registration import pose_from_xyz_rpy
        return pose_from_xyz_rpy(values)

    def cloud_cb(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.last_input = time.monotonic()
        try:
            if message.header.frame_id != self.sensor_frame:
                raise ValueError(f'unexpected sensor frame {message.header.frame_id!r}')
            scan = read_xyz(message)
            ranges = np.linalg.norm(scan, axis=1)
            scan = scan[np.isfinite(scan).all(axis=1) & (ranges >= self.min_range) & (ranges <= self.max_range)]
            scan = transform_points(scan, self.sensor_pose)
            result = self.mapper.update(scan)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.good_scans = 0
            self.reason = str(error)
            self.get_logger().warning(self.reason, throttle_duration_sec=3.)
            return
        self.reason = result.reason
        self.rmse = result.rmse if np.isfinite(result.rmse) else None
        self.fitness = result.fitness
        if not result.accepted:
            # Keep a recently accepted pose alive through one geometrically
            # rejected frame. The Mid-360 callback can briefly lag while the
            # vehicle is moving; clearing the three-scan warm-up counter on
            # every such frame created avoidable ExternalNav dropouts.
            self.get_logger().warning(f'Localization unavailable: {result.reason}', throttle_duration_sec=3.)
            return
        self.last_accepted = time.monotonic()
        self.good_scans += 1
        if self.good_scans < 3:
            return
        # Publish a short median for horizontal coordinates. A single scan can
        # lock onto a parallel wall with a low ICP residual; median gating
        # removes that one-frame XY spike. Keep the newest z sample because
        # Mid-360 callbacks are only a few hertz during takeoff and a full
        # three-frame median would leave the FCU behind the aircraft before
        # the next correction arrives.
        self._pose_samples.append(result.pose.copy())
        pose_result = result.pose.copy()
        if len(self._pose_samples) >= 2:
            translations = np.stack([pose[:3, 3] for pose in self._pose_samples])
            pose_result[:2, 3] = np.median(translations[:, :2], axis=0)
        if self._published_pose is not None:
            step = pose_result[:3, 3] - self._published_pose[:3, 3]
            step_norm = float(np.linalg.norm(step))
            if step_norm > 0.22:
                pose_result[:3, 3] = (
                    self._published_pose[:3, 3] + step * (0.22 / step_norm)
                )
        self._published_pose = pose_result.copy()
        # Preserve the LiDAR acquisition time.  The Gazebo bridge maps its
        # source clock into ROS time, so consumers can compare this estimate
        # with the truth/UGV streams at the same instant while the vehicle is
        # moving.  Fall back to receive time for drivers that leave the stamp
        # unset.
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.map_frame)
        pose = PoseStamped(header=header)
        xyz = pose_result[:3, 3]
        q = Rotation.from_matrix(pose_result[:3, :3]).as_quat()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, q)
        self.nav_pose_pub.publish(pose)
        transform = TransformStamped(header=header, child_frame_id='navigation_base')
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf.sendTransform(transform)
        self.path.header.stamp = header.stamp
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-self.path_length:]
        self.path_pub.publish(self.path)
        self.registered_pub.publish(make_cloud(transform_points(scan, result.pose), header))
        # Map is FLU; both the world and body conventions must change to NED/FRD.
        ned_pose = map_flu_to_ned(pose_result)
        ned_xyz = ned_pose[:3, 3]
        ned_q = Rotation.from_matrix(ned_pose[:3, :3]).as_quat()
        external = PoseWithCovarianceStamped()
        # Measurement time drives FCU/mission derivatives; RViz keeps ROS time.
        external.header.stamp = message.header.stamp
        external.header.frame_id = 'local_ned'
        (external.pose.pose.position.x, external.pose.pose.position.y,
         external.pose.pose.position.z) = map(float, ned_xyz)
        (external.pose.pose.orientation.x, external.pose.pose.orientation.y,
         external.pose.pose.orientation.z, external.pose.pose.orientation.w) = map(float, ned_q)
        external.pose.covariance = navigation_covariance(result.rmse)
        self.pose_pub.publish(external)
        self.cov_pub.publish(Float32(data=external.pose.covariance[0]))

    def health(self):
        now = time.monotonic()
        ready = (self.good_scans >= 3 and self.last_accepted is not None
                 and now - self.last_accepted <= self.stale_timeout)
        if self.last_input is None or now - self.last_input > self.stale_timeout:
            self.reason = 'cloud_stale'
            self.good_scans = 0
            ready = False
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'ready': ready, 'reason': self.reason,
            'rmse': self.rmse, 'fitness': round(self.fitness, 3),
            'map_points': 0 if self.mapper.points is None else len(self.mapper.points),
            'accepted_scans': self.mapper.accepted_scans,
        })))

    def publish_map(self):
        if self.mapper.points is not None:
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame)
            self.map_pub.publish(make_cloud(self.mapper.points, header))

    def save_cb(self, request, response):
        del request
        if self.mode != 'mapping':
            response.success = False
            response.message = 'Localization is read-only; its map is never updated or saved'
        elif self.mapper.accepted_scans < 4:
            response.success = False
            response.message = 'Need at least four accepted scans before saving'
        else:
            try:
                save_map(self.map_file, self.mapper.points, self.mapper.voxel_size, self.map_frame)
                response.success = True
                response.message = f'Saved {len(self.mapper.points)} points to {self.map_file} and PCD'
            except (ValueError, OSError) as error:
                response.success = False
                response.message = str(error)
        return response

    def autosave(self):
        if self.mode == 'mapping' and self.mapper.accepted_scans >= 4:
            response = self.save_cb(None, Trigger.Response())
            if not response.success:
                self.get_logger().error(response.message)


def main():
    rclpy.init()
    node = None
    try:
        node = PointCloudNavigation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.autosave()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
