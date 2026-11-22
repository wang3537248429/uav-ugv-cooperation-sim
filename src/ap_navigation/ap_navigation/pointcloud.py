"""XYZ conversion with explicit support for organized, padded PointCloud2 data."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def read_xyz(message):
    if message.width * message.height > 2_000_000:
        raise ValueError('point cloud exceeds configured reader capacity')
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    dtype_formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in formats or field.count != 1:
            raise ValueError('point cloud requires scalar floating-point XYZ fields')
        dtype_formats.append(endian + formats[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=dtype_formats,
                          offsets=offsets, itemsize=message.point_step))
    if message.row_step < message.width * message.point_step or len(message.data) < message.row_step * message.height:
        raise ValueError('truncated point cloud')
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step))
    return np.column_stack([values[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(float)


def make_cloud(points, header):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(points)
    message.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = bool(np.isfinite(points).all())
    message.data = points.tobytes()
    return message
