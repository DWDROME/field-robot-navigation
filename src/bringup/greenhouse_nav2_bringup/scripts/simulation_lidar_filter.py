#!/usr/bin/env python3
"""Remove Gazebo no-return points before the FAST-LIO generic XYZ handler.

Gazebo represents missing lidar returns with non-finite coordinates. The
generic FAST-LIO handler does not reliably reject them. Keep the original
fields, bytes, frame and acquisition time of every valid simulated return.
"""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def finite_returns(msg):
    if msg.width * msg.height > 1000000 or msg.point_step <= 0:
        raise ValueError('invalid or oversized simulated cloud')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('invalid simulated cloud layout')
    fields = {field.name: field for field in msg.fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('simulated coordinates must be scalar floating point')
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset + size > msg.point_step:
            raise ValueError('coordinate field exceeds point layout')
        formats.append(('>' if msg.is_bigendian else '<') + f'f{size}')
        offsets.append(field.offset)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    buffer = bytes(msg.data)
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=buffer,
                        strides=(msg.row_step, msg.point_step))
    keep = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    result = copy.copy(msg)
    raw = np.ndarray(points.shape, dtype=np.dtype(f'V{msg.point_step}'), buffer=buffer,
                     strides=(msg.row_step, msg.point_step))
    result.data = raw[keep].tobytes()
    result.height, result.width = 1, int(keep.sum())
    result.row_step = result.width * msg.point_step
    result.is_dense = True
    return result


class Filter(Node):
    def __init__(self):
        super().__init__('simulation_lidar_filter')
        self.publisher = self.create_publisher(PointCloud2, '/sensors/lidar/raw', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/simulation/lidar/unfiltered', self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        try:
            result = finite_returns(msg)
        except (KeyError, ValueError) as exc:
            self.get_logger().error(f'Rejected malformed simulated lidar: {exc}', throttle_duration_sec=5.)
            return
        if result.width:
            self.publisher.publish(result)


def main():
    rclpy.init()
    node = Filter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
