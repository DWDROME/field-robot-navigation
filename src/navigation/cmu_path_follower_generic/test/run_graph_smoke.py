#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GraphSmoke(Node):
    def __init__(self):
        super().__init__("path_follower_graph_smoke")
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/localization/odometry", reliable
        )
        self.path_pub = self.create_publisher(
            Path, "/planning/local_path", reliable
        )
        self.commands = []
        self.cmd_sub = self.create_subscription(
            TwistStamped, "/cmd_vel/nav", self.commands.append, reliable
        )

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def publish_path(self, empty=False):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        if not empty:
            for x in (0.0, 0.5, 1.0, 1.5, 2.0):
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.path_pub.publish(msg)


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    node = GraphSmoke()
    try:
        for _ in range(10):
            node.publish_odom()
            rclpy.spin_once(node, timeout_sec=0.03)
        node.publish_path()

        def forward_command_seen():
            node.publish_odom()
            node.publish_path()
            return any(
                command.twist.linear.x > 0.05
                and math.isfinite(command.twist.linear.x)
                and math.isfinite(command.twist.angular.z)
                and command.header.frame_id == "base_link"
                and (command.header.stamp.sec > 0 or command.header.stamp.nanosec > 0)
                for command in node.commands
            )

        if not spin_until(node, forward_command_seen, 5.0):
            raise AssertionError("no finite forward command arrived")

        publishers = node.get_publishers_info_by_topic("/cmd_vel/nav")
        if len(publishers) != 1 or publishers[0].node_name != "path_follower_generic":
            raise AssertionError(
                "unexpected command publishers: "
                f"{[(item.node_name, item.node_namespace) for item in publishers]!r}"
            )
        qos = publishers[0].qos_profile
        depth_is_explicit_or_unreported = qos.depth == 1 or (
            qos.depth == 0 and qos.history == HistoryPolicy.UNKNOWN
        )
        if (
            qos.reliability != ReliabilityPolicy.RELIABLE
            or not depth_is_explicit_or_unreported
        ):
            raise AssertionError(f"unexpected command QoS: {qos!r}")

        before_empty = len(node.commands)
        node.publish_path(empty=True)
        if not spin_until(
            node,
            lambda: any(
                command.twist.linear.x == 0.0 and command.twist.angular.z == 0.0
                for command in node.commands[before_empty:]
            ),
            2.0,
        ):
            raise AssertionError("empty path did not force a zero command")

        node.publish_odom()
        node.publish_path()
        if not spin_until(node, forward_command_seen, 3.0):
            raise AssertionError("follower did not recover after a new path")
        stale_start = len(node.commands)
        if not spin_until(
            node,
            lambda: any(
                command.twist.linear.x == 0.0 and command.twist.angular.z == 0.0
                for command in node.commands[stale_start:]
            ),
            2.0,
        ):
            raise AssertionError("stale odometry did not force a zero command")

        print(
            "PASS: forward tracking, sole-writer QoS, empty-path stop, "
            "and stale-odometry stop"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
