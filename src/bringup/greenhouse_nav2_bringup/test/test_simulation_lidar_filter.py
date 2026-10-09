"""Regression proof: remove no-return samples without corrupting other fields."""
import importlib.util
from pathlib import Path
import struct

import pytest
from sensor_msgs.msg import PointCloud2, PointField

path = Path(__file__).parents[1] / 'scripts' / 'simulation_lidar_filter.py'
spec = importlib.util.spec_from_file_location('simulation_lidar_filter', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('endian', ['<', '>'])
def test_no_return_filter_preserves_fields_padding_and_acquisition_time(endian):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar3d_0_laser'
    msg.header.stamp.sec = 42
    msg.is_bigendian = endian == '>'
    msg.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                  for index, name in enumerate(['x', 'y', 'z', 'intensity'])]
    msg.fields.append(PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1))
    first = struct.pack(endian+'ffffH', 1., 2., 3., 123., 7)+b'AB'
    invalid = struct.pack(endian+'ffffH', float('inf'), 2., 3., 555., 8)+b'CD'
    last = struct.pack(endian+'ffffH', 4., 5., 6., 999., 9)+b'EF'
    msg.height, msg.width, msg.point_step, msg.row_step = 2, 2, 20, 48
    msg.data = first+invalid+b'ROWPAD12'+invalid+last+b'ROWPAD34'
    output = module.finite_returns(msg)
    assert bytes(output.data) == first+last
    assert output.width == 2 and output.height == 1 and output.row_step == 40
    assert output.header == msg.header and output.fields == msg.fields
    assert output.is_dense and output.is_bigendian == msg.is_bigendian
