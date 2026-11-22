"""Inspect a saved map and check a preset route against observed obstacles."""

import argparse
import json

import numpy as np
from scipy.spatial import cKDTree

from .map_io import load_map
from .route_io import load_route


def route_clearance(points, route, clearance=0.18):
    if not np.isfinite(clearance) or clearance <= 0:
        raise ValueError('clearance must be positive')
    tree = cKDTree(points)
    closest = float('inf')
    for first, last in zip(route, route[1:]):
        a, b = np.array(first[:3]), np.array(last[:3])
        a[1] *= -1
        b[1] *= -1
        samples = np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / 0.025) + 1))
        distance, _ = tree.query(samples)
        minimum = float(distance.min())
        closest = min(closest, minimum)
        if minimum < clearance:
            raise ValueError(f'{first[3]} -> {last[3]}: observed obstacle within {minimum:.3f}m')
    return closest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_file')
    parser.add_argument('--route', help='Check route YAML against observed map surfaces')
    parser.add_argument('--clearance', type=float, default=0.18)
    args = parser.parse_args()
    try:
        points, voxel_size = load_map(args.map_file)
        result = {'points': len(points), 'voxel_size': voxel_size,
                  'min': points.min(axis=0).tolist(), 'max': points.max(axis=0).tolist()}
        if args.route:
            result['minimum_observed_clearance'] = route_clearance(points, load_route(args.route), args.clearance)
        print(json.dumps(result, indent=2))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'Map/route check failed: {error}\n')
