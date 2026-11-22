"""Small-scene 3D scan-to-map registration, independent of ROS and Gazebo.

Point-to-plane ICP estimates all six pose components. Mapping starts at an
explicit identity gauge; saved-map localization needs an approximate initial
pose. Neither mode reads geometry definitions, waypoints or ground truth.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def transform_points(points, pose):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]


def pose_from_xyz_rpy(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('initial_pose must contain six finite xyz/rpy values')
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    pose[:3, 3] = values[:3]
    return pose


def voxel_filter(points, size):
    """Use voxel centroids rather than the first (order-dependent) return."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if size <= 0 or not np.isfinite(size):
        raise ValueError('voxel size must be positive')
    if not len(points):
        return points
    cells = np.floor(points / size).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    centroids = np.zeros((len(counts), 3))
    np.add.at(centroids, inverse, points)
    return centroids / counts[:, None]


@dataclass
class RegistrationResult:
    pose: np.ndarray
    accepted: bool
    rmse: float
    fitness: float
    reason: str


class ScanMatcher:
    def __init__(self, max_distance=0.35, max_rmse=0.055, min_fitness=0.55,
                 max_translation=0.45, max_vertical_translation=0.35,
                 max_rotation=0.35, z_correction_gain=0.4,
                 z_correction_limit=0.30, iterations=8):
        self.max_distance = max_distance
        self.max_rmse = max_rmse
        self.min_fitness = min_fitness
        self.max_translation = max_translation
        self.max_vertical_translation = max_vertical_translation
        self.max_rotation = max_rotation
        # Floor/ceiling returns make point-to-plane ICP weakly observable in
        # z.  Apply only a damped, bounded point-to-point correction so one
        # bad wall correspondence cannot inject a full vertical jump.
        self.z_correction_gain = float(np.clip(z_correction_gain, 0.0, 1.0))
        self.z_correction_limit = max(0.05, float(z_correction_limit))
        self.iterations = iterations
        self.points = None
        self.tree = None

    def set_map(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 80 or not np.isfinite(points).all():
            raise ValueError('map requires at least 80 finite XYZ points')
        self.points = points.copy()
        self.tree = cKDTree(self.points)
        _, neighbors = self.tree.query(self.points, k=min(12, len(points)))
        neighborhoods = self.points[neighbors]
        centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / neighborhoods.shape[1]
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        self.normals = eigenvectors[:, :, 0]
        self.planar = (eigenvalues[:, 1] > 1e-6) & (
            eigenvalues[:, 0] < 0.18 * np.maximum(eigenvalues[:, 1], 1e-9)
        )

    def match(self, scan, initial):
        initial = np.asarray(initial, dtype=float)
        pose = initial.copy()
        scan = np.asarray(scan, dtype=float).reshape(-1, 3)
        scan = scan[np.isfinite(scan).all(axis=1)]
        if self.tree is None or len(scan) < 80:
            return RegistrationResult(initial.copy(), False, float('inf'), 0.0, 'too_few_points')
        rank = 0
        for _ in range(self.iterations):
            world = transform_points(scan, pose)
            distance, indices = self.tree.query(world, distance_upper_bound=self.max_distance)
            mask = np.isfinite(distance)
            mask[mask] &= self.planar[indices[mask]]
            if mask.sum() < 60:
                return RegistrationResult(initial.copy(), False, float('inf'), 0.0, 'low_overlap')
            p = world[mask]
            q = self.points[indices[mask]]
            normals = self.normals[indices[mask]]
            residual = np.einsum('ij,ij->i', normals, p - q)
            # Huber weights reduce moving objects and edge correspondences.
            weight = np.sqrt(np.minimum(1.0, 0.035 / np.maximum(abs(residual), 1e-9)))
            jacobian = np.column_stack((np.cross(p, normals), normals))
            delta, _, rank, _ = np.linalg.lstsq(
                jacobian * weight[:, None], -residual * weight, rcond=1e-4
            )
            if rank < 6:
                return RegistrationResult(initial.copy(), False, float('inf'), 0.0, 'degenerate_geometry')
            # Refuse unstable optimization steps instead of corrupting the map.
            # Mid-360 clouds can change by more than 30 cm during the
            # constrained takeoff/landing manoeuvre. Keep a bounded but
            # realistic step guard; the final pose_jump check below still
            # rejects a complete registration loss.
            if np.linalg.norm(delta[:3]) > 0.6 or np.linalg.norm(delta[3:]) > 0.6:
                return RegistrationResult(initial.copy(), False, float('inf'), 0.0, 'optimizer_jump')
            rotation = Rotation.from_rotvec(delta[:3]).as_matrix()
            pose[:3, :3] = rotation @ pose[:3, :3]
            pose[:3, 3] = rotation @ pose[:3, 3] + delta[3:]
            if np.linalg.norm(delta) < 1e-5:
                break
        world = transform_points(scan, pose)
        distance, indices = self.tree.query(world, distance_upper_bound=self.max_distance)
        mask = np.isfinite(distance)
        mask[mask] &= self.planar[indices[mask]]
        # Point-to-plane ICP is intentionally conservative about the z axis
        # in a room with a large parallel floor/ceiling. Recover that missing
        # component from robust point-to-point correspondence heights. This
        # keeps a slow Mid-360 takeoff observable without using simulator
        # truth; outliers are bounded and the normal/fitness checks below are
        # still applied after the correction.
        zmask = np.isfinite(distance)
        if zmask.sum() >= 60:
            # Restrict the scalar correction to horizontal map planes. Wall
            # returns are largely invariant under a vertical translation and
            # dominated the previous all-return median, making takeoff z
            # appear frozen. Floor/ceiling normals provide the observable
            # component while the six-DoF solve remains unchanged.
            horizontal = np.zeros_like(zmask)
            horizontal[zmask] = np.abs(self.normals[indices[zmask], 2]) > 0.70
            if horizontal.sum() >= 30:
                zmask = horizontal
            z_residual = float(np.median(self.points[indices[zmask], 2] - world[zmask, 2]))
            if (np.isfinite(z_residual)
                    and 0.02 < abs(z_residual) <= self.z_correction_limit):
                correction = self.z_correction_gain * z_residual
                pose[2, 3] += float(np.clip(correction, -0.16, 0.16))
                world = transform_points(scan, pose)
                distance, indices = self.tree.query(world, distance_upper_bound=self.max_distance)
                mask = np.isfinite(distance)
                mask[mask] &= self.planar[indices[mask]]
        fitness = float(mask.sum() / len(scan))
        residual = np.einsum('ij,ij->i', self.normals[indices[mask]], world[mask] - self.points[indices[mask]])
        rmse = float(np.sqrt(np.mean(residual ** 2))) if len(residual) else float('inf')
        movement = np.linalg.inv(initial) @ pose
        translation = np.linalg.norm(movement[:3, 3])
        angle = Rotation.from_matrix(movement[:3, :3]).magnitude()
        vertical = abs(float(movement[2, 3]))
        if (translation > self.max_translation
                or vertical > self.max_vertical_translation
                or angle > self.max_rotation):
            reason = 'pose_jump'
        elif fitness < self.min_fitness:
            reason = 'low_overlap'
        elif rmse > self.max_rmse:
            reason = 'high_residual'
        else:
            reason = 'ok'
        return RegistrationResult(pose if reason == 'ok' else initial.copy(), reason == 'ok', rmse, fitness, reason)


class PointCloudMapper:
    def __init__(self, mode='mapping', points=None, initial_pose=None, voxel_size=0.06,
                 max_vertical_step=0.12, max_scan_points=3200,
                 upper_structure_quantile=0.0, **matcher_options):
        if mode not in ('mapping', 'localization'):
            raise ValueError('mode must be mapping or localization')
        if mode == 'localization' and points is None:
            raise ValueError('localization requires a saved map')
        self.mode = mode
        self.voxel_size = voxel_size
        self.max_scan_points = max(400, int(max_scan_points))
        self.max_vertical_step = max(0.03, float(max_vertical_step))
        self.upper_structure_quantile = float(upper_structure_quantile)
        if self.upper_structure_quantile and not 0.95 <= self.upper_structure_quantile < 1.0:
            raise ValueError('upper_structure_quantile must be zero (disabled) or in [0.95, 1)')
        self.pose = pose_from_xyz_rpy(initial_pose if initial_pose is not None else [0.] * 6)
        self.matcher = ScanMatcher(**matcher_options)
        self.points = None
        self.keyframe_pose = None
        self.accepted_scans = 0
        # Optional prior for this arena's fixed, level wall tops. Mid-360
        # sees too little floor during ascent to constrain Z reliably. The
        # prior seeds ICP from the observed upper envelope; it is disabled
        # for general scenes with uneven or changing upper structures.
        self._upper_structure_reference = None
        if points is not None:
            self.points = voxel_filter(points, voxel_size)
            self.matcher.set_map(self.points)
            self.keyframe_pose = self.pose.copy()
            self._upper_structure_reference = self._upper_structure_level(self.points)

    def _upper_structure_level(self, points):
        """Return an observed upper return level, when the arena prior is enabled."""
        if not self.upper_structure_quantile:
            return None
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 80:
            return None
        values = points[:, 2]
        values = values[np.isfinite(values)]
        if len(values) < 80:
            return None
        threshold = float(np.quantile(values, self.upper_structure_quantile))
        upper = values[values >= threshold]
        return float(np.median(upper)) if len(upper) else threshold

    def update(self, scan):
        scan = voxel_filter(scan, self.voxel_size)
        # A Mid-360 frame is dense enough that matching every return in the
        # Python node makes the 10 Hz stream back up. Keep a deterministic,
        # spatially spread subset for ICP; the saved map still retains the
        # full first frame and keyframes remain bounded by voxel filtering.
        if len(scan) > self.max_scan_points:
            indices = np.linspace(0, len(scan) - 1, self.max_scan_points, dtype=np.int64)
            scan = scan[indices]
        if len(scan) < 80:
            return RegistrationResult(self.pose.copy(), False, float('inf'), 0., 'too_few_points')
        if self.points is None:
            self.points = transform_points(scan, self.pose)
            self.matcher.set_map(self.points)
            self.keyframe_pose = self.pose.copy()
            self.accepted_scans = 1
            self._upper_structure_reference = self._upper_structure_level(self.points)
            # A first scan defines the map, but is not yet a localization check.
            return RegistrationResult(self.pose.copy(), False, 0., 1., 'initializing')
        orientation = self.pose.copy()
        orientation[:3, 3] = 0.0
        upper_structure = self._upper_structure_level(transform_points(scan, orientation))
        prior_pose_z = None
        if self._upper_structure_reference is not None and upper_structure is not None:
            candidate = self._upper_structure_reference - upper_structure
            # Reject a prior outside the bounded recovery window. ICP's
            # overlap, residual and rotation checks still apply to the seed.
            if np.isfinite(candidate) and abs(candidate - self.pose[2, 3]) <= max(
                0.8, 2.0 * self.max_vertical_step
            ):
                prior_pose_z = float(candidate)
        initial = self.pose.copy()
        if prior_pose_z is not None:
            initial[2, 3] = prior_pose_z
        result = self.matcher.match(scan, initial)
        if not result.accepted:
            return result
        # A parallel floor/ceiling can produce a plausible ICP solution with
        # a larger z change than the 10 Hz flight profile can physically make.
        # Clamp that single-frame innovation instead of dropping ExternalNav
        # completely; the next scan can then catch up without a stale-pose
        # fault in the flight controller.
        dz = float(result.pose[2, 3] - self.pose[2, 3])
        if abs(dz) > self.max_vertical_step:
            pose = result.pose.copy()
            pose[2, 3] = self.pose[2, 3] + np.clip(
                dz, -self.max_vertical_step, self.max_vertical_step
            )
            result = RegistrationResult(pose, True, result.rmse, result.fitness,
                                        'ok_vertical_clamped')
        self.pose = result.pose.copy()
        self.accepted_scans += 1
        if self.mode == 'mapping':
            motion = np.linalg.inv(self.keyframe_pose) @ self.pose
            if (np.linalg.norm(motion[:3, 3]) > 0.12
                    or Rotation.from_matrix(motion[:3, :3]).magnitude() > 0.08):
                self.points = voxel_filter(np.vstack((self.points, transform_points(scan, self.pose))), self.voxel_size)
                self.matcher.set_map(self.points)
                self.keyframe_pose = self.pose.copy()
        return result
