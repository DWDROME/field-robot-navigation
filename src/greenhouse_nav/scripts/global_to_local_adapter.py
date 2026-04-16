#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32


DEFAULT_STATE_ESTIMATION_TOPIC = '/state_estimation'
DEFAULT_GLOBAL_PATH_TOPIC = '/waypoints'
DEFAULT_LOCAL_WAYPOINT_TOPIC = '/way_point'
DEFAULT_SPEED_TOPIC = '/speed'
DEFAULT_DEBUG_PATH_TOPIC = '/adapter/waypoint_window'


def point_distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class GlobalToLocalAdapter(object):
    def __init__(self):
        self.publish_rate_hz = rospy.get_param('~publish_rate_hz', 20.0)
        self.look_ahead_distance = rospy.get_param('~look_ahead_distance', 1.5)
        self.goal_reached_tolerance = rospy.get_param('~goal_reached_tolerance', 0.3)
        self.default_speed = rospy.get_param('~default_speed', 0.4)
        self.stop_speed = rospy.get_param('~stop_speed', 0.0)
        self.publish_debug_path = rospy.get_param('~publish_debug_path', True)

        self.state_estimation_topic = rospy.get_param('~state_estimation_topic', DEFAULT_STATE_ESTIMATION_TOPIC)
        self.global_path_topic = rospy.get_param('~global_path_topic', DEFAULT_GLOBAL_PATH_TOPIC)
        self.local_waypoint_topic = rospy.get_param('~local_waypoint_topic', DEFAULT_LOCAL_WAYPOINT_TOPIC)
        self.speed_topic = rospy.get_param('~speed_topic', DEFAULT_SPEED_TOPIC)
        self.debug_path_topic = rospy.get_param('~debug_path_topic', DEFAULT_DEBUG_PATH_TOPIC)

        self._lock = threading.Lock()
        self._state_estimation = None
        self._global_path = None

        self._waypoint_publisher = rospy.Publisher(self.local_waypoint_topic, PointStamped, queue_size=1)
        self._speed_publisher = rospy.Publisher(self.speed_topic, Float32, queue_size=1)
        self._debug_path_publisher = rospy.Publisher(self.debug_path_topic, Path, queue_size=1)

        rospy.Subscriber(self.state_estimation_topic, Odometry, self._state_estimation_callback, queue_size=1)
        rospy.Subscriber(self.global_path_topic, Path, self._global_path_callback, queue_size=1)

    def _state_estimation_callback(self, odom_msg):
        with self._lock:
            self._state_estimation = odom_msg

    def _global_path_callback(self, path_msg):
        with self._lock:
            self._global_path = path_msg

    def _snapshot(self):
        with self._lock:
            state_estimation = copy.copy(self._state_estimation)
            global_path = copy.deepcopy(self._global_path)
        return state_estimation, global_path

    @staticmethod
    def _resolve_path_frame(global_path, state_estimation):
        if global_path.header.frame_id:
            return global_path.header.frame_id
        if state_estimation is not None:
            return state_estimation.header.frame_id
        return ''

    def _publish_stop(self):
        self._speed_publisher.publish(Float32(data=self.stop_speed))

    def _publish_empty_debug_path(self, frame_id):
        if not self.publish_debug_path:
            return
        debug_path = Path()
        debug_path.header.stamp = rospy.Time.now()
        debug_path.header.frame_id = frame_id
        self._debug_path_publisher.publish(debug_path)

    def _select_target_index(self, current_point, poses):
        nearest_index = 0
        nearest_distance = float('inf')
        for idx, pose_stamped in enumerate(poses):
            distance = point_distance(current_point, pose_stamped.pose.position)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = idx

        target_index = nearest_index
        accumulated_distance = nearest_distance
        while target_index < len(poses) - 1 and accumulated_distance < self.look_ahead_distance:
            accumulated_distance += point_distance(
                poses[target_index].pose.position,
                poses[target_index + 1].pose.position,
            )
            target_index += 1

        final_distance = point_distance(current_point, poses[-1].pose.position)
        goal_reached = final_distance <= self.goal_reached_tolerance
        if goal_reached:
            target_index = len(poses) - 1

        return nearest_index, target_index, goal_reached

    def _publish_debug_path(self, header, poses, nearest_index, target_index):
        if not self.publish_debug_path:
            return
        debug_path = Path()
        debug_path.header.stamp = header.stamp
        debug_path.header.frame_id = header.frame_id
        debug_path.poses = poses[nearest_index:target_index + 1]
        self._debug_path_publisher.publish(debug_path)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            state_estimation, global_path = self._snapshot()

            if state_estimation is None:
                rospy.logwarn_throttle(2.0, 'GlobalToLocalAdapter waiting for /state_estimation.')
                self._publish_stop()
                self._publish_empty_debug_path('')
                rate.sleep()
                continue

            if global_path is None or len(global_path.poses) == 0:
                rospy.logwarn_throttle(2.0, 'GlobalToLocalAdapter waiting for non-empty global path.')
                self._publish_stop()
                self._publish_empty_debug_path(state_estimation.header.frame_id)
                rate.sleep()
                continue

            path_frame_id = self._resolve_path_frame(global_path, state_estimation)
            if global_path.header.frame_id and state_estimation.header.frame_id and \
                    global_path.header.frame_id != state_estimation.header.frame_id:
                rospy.logwarn_throttle(
                    2.0,
                    'GlobalToLocalAdapter frame mismatch: path=%s state=%s',
                    global_path.header.frame_id,
                    state_estimation.header.frame_id,
                )
                self._publish_stop()
                self._publish_empty_debug_path(path_frame_id)
                rate.sleep()
                continue

            current_point = state_estimation.pose.pose.position
            nearest_index, target_index, goal_reached = self._select_target_index(current_point, global_path.poses)

            waypoint = PointStamped()
            waypoint.header.stamp = state_estimation.header.stamp
            waypoint.header.frame_id = path_frame_id
            waypoint.point = copy.deepcopy(global_path.poses[target_index].pose.position)

            self._waypoint_publisher.publish(waypoint)
            target_speed = self.stop_speed if goal_reached else self.default_speed
            self._speed_publisher.publish(Float32(data=target_speed))
            self._publish_debug_path(waypoint.header, global_path.poses, nearest_index, target_index)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('global_to_local_adapter')
    rospy.loginfo('Global-To-Local Adapter Node Inited...')
    GlobalToLocalAdapter().spin()
