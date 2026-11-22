"""Catch wall crossings, whole-body clipping and unstable round marker IDs."""

from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from ap_simulation.competition_paths import UGV_PATH_GAZEBO
from ap_simulation.random_marker import marker_sdf, next_marker_id
from ap_simulation.ugv_geometry import check_pose, load_obstacles, rectangle, validate_path


ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / 'src/ap_simulation/worlds/competition_world.sdf'


def test_route_and_full_turning_footprint_clear_actual_world_collisions():
    assert validate_path(load_obstacles(WORLD)) > 700
    assert max(p[0] for p in UGV_PATH_GAZEBO) <= 2.5
    old_route = [(-2.25, 1.8), (3.1, 2.), (3.3, 1.5), (3.1, -2.05), (-2.25, -1.85)]
    with pytest.raises(ValueError, match='footprint intersects'):
        validate_path(load_obstacles(WORLD), points=old_route)


def test_clear_waypoint_centres_do_not_hide_body_clipping_or_turn_collisions():
    obstacles = [('wall', rectangle(1., 0., 0., .05, 2.))]
    with pytest.raises(ValueError, match='footprint'):
        check_pose(.8, 0., 0., obstacles)
    with pytest.raises(ValueError, match='wall'):
        validate_path(obstacles, points=[(0., 0.), (2., 0.)])


def test_random_marker_selection_covers_valid_ids_without_repeating_previous():
    for previous in range(50):
        outcomes = {next_marker_id(previous, randbelow=lambda bound, n=n: n) for n in range(49)}
        assert outcomes == set(range(50)) - {previous}


def test_gazebo_marker_texture_contains_the_selected_id(tmp_path):
    sdf = marker_sdf(37, tmp_path, ROOT / 'src/ap_simulation/models/aruco_marker/model.sdf')
    root = ET.fromstring(sdf)
    assert root.find('model').get('name') == 'round_aruco'
    texture = cv2.imread(root.findtext('.//albedo_map'), cv2.IMREAD_GRAYSCALE)
    padded = np.pad(texture, 25, constant_values=255)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(padded)
    assert ids.flatten().tolist() == [37]
