"""Check that PX4 integer mode constants survive ROS serialization."""
from rclpy.serialization import deserialize_message, serialize_message
from px4_msgs.msg import VehicleCommand
from greenhouse_sim_air.air_mission import AirMission


def test_mode_parameters_survive_serialization():
    class Publisher:
        def publish(self, message):
            self.message = message

    class Mission:
        command_pub = Publisher()

        def now_us(self):
            return 123456

    mission = Mission()
    AirMission.send_command(mission, 176, 1, 4, 6)
    received = deserialize_message(
        serialize_message(mission.command_pub.message), VehicleCommand)
    assert (received.command, received.param1, received.param2, received.param3) == (
        176, 1.0, 4.0, 6.0)
    assert received.timestamp == 123456
    assert received.from_external
