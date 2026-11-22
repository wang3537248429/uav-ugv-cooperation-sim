"""Registration accuracy, map persistence and failure modes without a simulator."""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from ap_mission.competition_route import COMPETITION_WAYPOINTS_NED
from ap_navigation.frames import map_flu_to_ned, navigation_covariance
from ap_navigation.map_io import load_map, save_map
from ap_navigation.map_tool import route_clearance
from ap_navigation.registration import PointCloudMapper, ScanMatcher, pose_from_xyz_rpy, transform_points
from ap_navigation.route_io import load_route


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def room():
    rng = np.random.default_rng(12)
    points = rng.uniform([-2., -2.5, -.3], [3., 2.5, 2.5], size=(6000, 3))
    for i in range(6):
        axis = i // 2
        points[i * 1000:(i + 1) * 1000, axis] = [-2., -2.5, -.3][axis] if i % 2 == 0 else [3., 2.5, 2.5][axis]
    return points


def test_registration_recovers_six_dof_from_noisy_scan_with_outliers(room):
    matcher = ScanMatcher()
    matcher.set_map(room)
    truth = pose_from_xyz_rpy([.12, -.08, .18, .02, -.015, .07])
    rng = np.random.default_rng(5)
    scan = transform_points(room, np.linalg.inv(truth)) + rng.normal(0, .003, room.shape)
    scan = np.vstack((scan, rng.uniform(-3, 3, (500, 3))))
    result = matcher.match(scan, np.eye(4))
    assert result.accepted, result.reason
    assert np.linalg.norm(result.pose[:3, 3] - truth[:3, 3]) < .008
    assert Rotation.from_matrix(result.pose[:3, :3].T @ truth[:3, :3]).magnitude() < .004


def test_mapping_loop_then_restart_localization_uses_saved_map_unchanged(room, tmp_path):
    mapper = PointCloudMapper()
    rng = np.random.default_rng(7)
    for step in list(range(9)) + list(range(8, -1, -1)):
        truth = pose_from_xyz_rpy([step * .045, step * -.02, step * .025, 0, 0, step * .008])
        scan = transform_points(room, np.linalg.inv(truth)) + rng.normal(0, .002, room.shape)
        result = mapper.update(scan)
        assert result.accepted or result.reason == 'initializing'
    assert np.linalg.norm(mapper.pose[:3, 3]) < .012
    filename = save_map(tmp_path / 'room.npz', mapper.points, mapper.voxel_size)
    file_before = filename.read_bytes()
    points, resolution = load_map(filename)
    localizer = PointCloudMapper(
        mode='localization', points=points, voxel_size=resolution, initial_pose=[.1, -.08, .04, 0, 0, .03])
    before = localizer.points.copy()
    truth = pose_from_xyz_rpy([.18, -.1, .12, .01, 0, .08])
    for _ in range(4):
        result = localizer.update(transform_points(room, np.linalg.inv(truth)))
        assert result.accepted
    assert np.linalg.norm(result.pose[:3, 3] - truth[:3, 3]) < .015
    np.testing.assert_array_equal(localizer.points, before)
    assert filename.read_bytes() == file_before
    assert filename.with_suffix('.pcd').read_bytes().startswith(b'# .PCD v0.7')


def test_single_plane_is_rejected_instead_of_publishing_ambiguous_pose():
    xy = np.random.default_rng(2).uniform(-2., 2., (800, 2))
    plane = np.column_stack((xy, np.zeros(800)))
    matcher = ScanMatcher()
    matcher.set_map(plane)
    result = matcher.match(plane, np.eye(4))
    assert not result.accepted
    assert result.reason == 'degenerate_geometry'


def test_missing_overlap_and_pose_jumps_do_not_update_map(room):
    mapper = PointCloudMapper()
    mapper.update(room)
    before = mapper.points.copy()
    result = mapper.update(room + 50.)
    assert not result.accepted
    np.testing.assert_array_equal(mapper.points, before)
    np.testing.assert_array_equal(mapper.pose, np.eye(4))
    assert not mapper.update(np.full((200, 3), np.nan)).accepted


def test_map_validation_rejects_wrong_frame_and_invalid_points(room, tmp_path):
    path = save_map(tmp_path / 'map.npz', room, .06, 'other_frame')
    with pytest.raises(ValueError, match='frame'):
        load_map(path)
    with pytest.raises(ValueError):
        save_map(tmp_path / 'bad.npz', np.zeros((2, 3)), .06)
    with pytest.raises(ValueError):
        PointCloudMapper(mode='localization')


def test_editable_route_preserves_ned_contract_and_rejects_invalid_tasks(tmp_path):
    path = ROOT / 'src/ap_navigation/config/competition_waypoints.yaml'
    assert load_route(path) == COMPETITION_WAYPOINTS_NED
    data = yaml.safe_load(path.read_text())
    data['waypoints'][2]['position'][1] = 0.25
    edited = tmp_path / 'route.yaml'
    edited.write_text(yaml.safe_dump(data))
    assert load_route(edited)[2][1] == -.25
    data['waypoints'][2]['label'] = 'WRONG_TASK'
    edited.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_route(edited)


def test_route_accepts_transit_points_but_rejects_ignored_post_balloon_points(tmp_path):
    data = yaml.safe_load((ROOT / 'src/ap_navigation/config/competition_waypoints.yaml').read_text())
    data['waypoints'].insert(3, {'label': 'RETURN_BEND', 'position': [2., .3, 1.]})
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump(data))
    assert load_route(path)[3] == (2., -.3, 1., 'RETURN_BEND')
    data['waypoints'].insert(-1, {'label': 'HOME_BEND', 'position': [.3, -.8, 1.]})
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='fixed corridor'):
        load_route(path)


def test_route_clearance_checks_segments_not_only_waypoint_endpoints():
    route = [(0., 0., 1., 'A'), (2., 0., 1., 'B')]
    with pytest.raises(ValueError, match='A -> B'):
        route_clearance(np.array([[1., 0., 1.]]), route, .2)
    assert route_clearance(np.array([[1., 1., 1.]]), route, .2) > .9


def test_external_nav_frame_and_combined_angular_uncertainty():
    pose = pose_from_xyz_rpy([1., 2., 3., .1, -.2, .3])
    ned = map_flu_to_ned(pose)
    np.testing.assert_allclose(ned[:3, 3], [1., -2., -3.])
    np.testing.assert_allclose(Rotation.from_matrix(ned[:3, :3]).as_euler('xyz'), [.1, .2, -.3])
    covariance = navigation_covariance(.02)
    # The pinned ArduPilot MAVLink handler uses cbrt(sum(angular_var^2)).
    combined_angle_error = sum(covariance[i] ** 2 for i in (21, 28, 35)) ** (1. / 3.)
    assert combined_angle_error < .05


def test_mid360_ascent_with_sparse_floor_keeps_vertical_pose_bounded():
    """Ray-cast an open-topped arena, using only Mid-360's limited lower FOV."""
    azimuth, elevation = np.meshgrid(np.linspace(-np.pi, np.pi, 600),
                                     np.deg2rad(np.linspace(-7., 52., 32)))
    directions = np.column_stack((
        (np.cos(elevation) * np.cos(azimuth)).ravel(),
        (np.cos(elevation) * np.sin(azimuth)).ravel(), np.sin(elevation).ravel(),
    ))

    def cloud(altitude):
        origin = np.array([-2.25, 0., .39 + altitude])
        distances = []
        for axis, boundary in ((0, -3.), (0, 3.), (1, -2.5), (1, 2.5), (2, 0.)):
            component = directions[:, axis]
            distance = (boundary - origin[axis]) / np.where(abs(component) > 1e-9, component, 1e-9)
            hit = origin + directions * distance[:, None]
            inside = (distance > 0.) & np.all(hit >= [-3.00001, -2.50001, -.00001], axis=1)
            inside &= np.all(hit <= [3.00001, 2.50001, 2.55001], axis=1)
            distances.append(np.where(inside, distance, np.inf))
        nearest = np.min(distances, axis=0)
        valid = np.isfinite(nearest)
        points = directions[valid] * nearest[valid, None]
        points[:, 2] += .15  # sensor -> body extrinsic
        return points + np.random.default_rng(5).normal(0., .002, points.shape)

    mapper = PointCloudMapper(
        upper_structure_quantile=.995, max_distance=.5, min_fitness=.45,
        max_rmse=.08, max_translation=.55, max_vertical_translation=.8,
        max_vertical_step=.2, max_scan_points=2400,
    )
    for altitude in [0., 0.] + list(np.linspace(0., 1., 11)) + [1., 1.]:
        result = mapper.update(cloud(altitude))
        assert result.accepted or result.reason == 'initializing'
        assert abs(result.pose[2, 3] - altitude) < .15
        assert np.linalg.norm(result.pose[:2, 3]) < .02
    before = mapper.points.copy()
    assert not mapper.update(cloud(1.) + 50.).accepted
    np.testing.assert_array_equal(mapper.points, before)


@pytest.mark.parametrize('quantile', [-.1, .5, 1., float('nan')])
def test_upper_structure_prior_rejects_invalid_configuration(quantile):
    with pytest.raises(ValueError, match='upper_structure_quantile'):
        PointCloudMapper(upper_structure_quantile=quantile)
