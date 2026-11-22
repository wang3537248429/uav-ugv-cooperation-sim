"""Versioned, non-pickle point-cloud maps and portable PCD export."""

import os
from pathlib import Path
import tempfile

import numpy as np


def save_map(path, points, voxel_size, frame_id='uav_map'):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float32)
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError('map resolution must be positive')
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 80 or not np.isfinite(points).all():
        raise ValueError('refusing to save an empty or invalid map')
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            np.savez_compressed(stream, schema=np.array(1), points=points,
                                voxel_size=np.array(voxel_size), frame_id=np.array(frame_id))
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    pcd = path.with_suffix('.pcd')
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    with pcd.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.astype('<f4').tobytes())
    return path


def load_map(path, expected_frame='uav_map'):
    with np.load(Path(path).expanduser(), allow_pickle=False) as data:
        if int(data['schema']) != 1:
            raise ValueError('unsupported map schema')
        if str(data['frame_id']) != expected_frame:
            raise ValueError('map frame does not match navigation frame')
        points = np.asarray(data['points'], dtype=float)
        voxel_size = float(data['voxel_size'])
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 80 or not np.isfinite(points).all():
        raise ValueError('invalid point-cloud map')
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError('invalid map resolution')
    return points, voxel_size
