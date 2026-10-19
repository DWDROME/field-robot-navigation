"""A discovered PX4 topic must not start a mission before telemetry arrives."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs')
pytest.importorskip('greenhouse_sim_air')
from greenhouse_sim_air.air_mission import output_topic, px4_qos
from px4_msgs.msg import VehicleLocalPosition

path = Path(__file__).parents[1]/'scripts/run_simulation_run.py'
spec = importlib.util.spec_from_file_location('simulation_runner', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_waits_for_actual_message_after_topic_discovery(tmp_path, monkeypatch):
    # Synthetic interface traffic is isolated from the physical simulations.
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    marker = tmp_path/'publish-now'
    code = '''
import sys,time,rclpy
from pathlib import Path
from px4_msgs.msg import VehicleLocalPosition
from greenhouse_sim_air.air_mission import output_topic,px4_qos
rclpy.init(); node=rclpy.create_node('px4_startup_test_publisher')
publisher=node.create_publisher(VehicleLocalPosition,
    output_topic('vehicle_local_position',VehicleLocalPosition),px4_qos())
message=VehicleLocalPosition(); message.timestamp=1
print('ready',flush=True)
while True:
    if Path(sys.argv[1]).exists(): publisher.publish(message)
    rclpy.spin_once(node,timeout_sec=.05)
'''
    publisher = subprocess.Popen([sys.executable, '-c', code, str(marker)],
                                 stdout=subprocess.PIPE, text=True, env=dict(os.environ))
    try:
        assert publisher.stdout.readline().strip() == 'ready'
        rclpy.init()
        node = rclpy.create_node('px4_startup_test_discovery')
        try:
            topic = output_topic('vehicle_local_position', VehicleLocalPosition)
            deadline = time.monotonic()+5
            while node.count_publishers(topic) == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert node.count_publishers(topic) == 1
        finally:
            node.destroy_node()
            rclpy.shutdown()
        assert not runner.wait_for_px4_position(.5)
        marker.touch()
        assert runner.wait_for_px4_position(5)
    finally:
        publisher.kill()
        publisher.wait(timeout=5)
