#!/usr/bin/env python3
"""Wait for physical settling before FAST-LIO samples its gravity reference.

Gazebo IMUs report zero specific force during the spawn's free fall. Starting
FAST-LIO then can normalize a near-zero gravity sample and diverge. This
SIM-only startup check observes actual IMU, contacts and clock; it neither
changes sensor data nor authorizes navigation or movement.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu


def main():
    rclpy.init()
    node = rclpy.create_node('simulation_ground_startup')
    node.declare_parameter('output', '')
    state = {'clock': None, 'clock_receipt': -math.inf,
             'imu_receipt': -math.inf, 'contact_receipt': -math.inf,
             'stable_since': None, 'acceleration_norm': None, 'angular_speed': None}
    def clock(msg):
        state.update(clock=msg.clock.sec+msg.clock.nanosec*1e-9,
                     clock_receipt=time.monotonic())
    def contact(msg):
        if msg.contacts:
            state['contact_receipt'] = time.monotonic()
    def imu(msg):
        acceleration, angular = msg.linear_acceleration, msg.angular_velocity
        values = [acceleration.x, acceleration.y, acceleration.z, angular.x, angular.y, angular.z]
        if not all(math.isfinite(value) for value in values) or state['clock'] is None:
            state['stable_since'] = None
            return
        norm = math.sqrt(sum(value*value for value in values[:3]))
        speed = math.sqrt(sum(value*value for value in values[3:]))
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        settled = (abs(norm-9.81) < .5 and speed < .1 and
                   -.1 <= state['clock']-stamp <= .2 and
                   time.monotonic()-state['contact_receipt'] < .7)
        state.update(imu_receipt=time.monotonic(), acceleration_norm=norm, angular_speed=speed)
        if not settled:
            state['stable_since'] = None
        elif state['stable_since'] is None:
            state['stable_since'] = state['clock']
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(Contacts, '/simulation/contacts', contact, 10)
    node.create_subscription(Imu, '/sensors/imu/data', imu, qos_profile_sensor_data)
    deadline = time.monotonic()+120
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now-state['imu_receipt'] > .5 or now-state['clock_receipt'] > .5:
                state['stable_since'] = None
            if (state['stable_since'] is not None and
                    state['clock']-state['stable_since'] >= 1 and
                    node.count_publishers('/clock') == 1):
                result = {'result': 'settled', 'sim_time_s': state['clock'],
                          'stable_sim_s': state['clock']-state['stable_since'],
                          'acceleration_norm_mps2': state['acceleration_norm'],
                          'angular_speed_radps': state['angular_speed'],
                          'clock_publishers': 1}
                output = node.get_parameter('output').value
                if output:
                    Path(output).write_text(json.dumps(result, indent=2))
                print(json.dumps(result), flush=True)
                return 0
        print('Physical settling was not observed within 120 wall seconds', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
