#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


OUTPUT_FRAME_ID = 'map'
OUTPUT_CHILD_FRAME_ID = 'base_link'
DEFAULT_INPUT_ODOM_TOPIC = '/Odometry'
DEFAULT_MAP_TO_ODOM_TOPIC = '/map_to_odom'
DEFAULT_OUTPUT_TOPIC = '/state_estimation'


class LocalizationManager(object):
    def __init__(self):
        self.publish_rate_hz = rospy.get_param('~publish_rate_hz', 50.0)
        self.input_odom_topic = rospy.get_param('~input_odom_topic', DEFAULT_INPUT_ODOM_TOPIC)
        self.map_to_odom_topic = rospy.get_param('~map_to_odom_topic', DEFAULT_MAP_TO_ODOM_TOPIC)
        self.output_topic = rospy.get_param('~output_topic', DEFAULT_OUTPUT_TOPIC)
        self.output_frame_id = rospy.get_param('~output_frame_id', OUTPUT_FRAME_ID)
        self.output_child_frame_id = rospy.get_param('~output_child_frame_id', OUTPUT_CHILD_FRAME_ID)
        self.broadcast_tf = rospy.get_param('~broadcast_tf', True)
        self.body_to_base_mat = self._build_body_to_base_mat()

        self._lock = threading.Lock()
        self._cur_odom_to_base = None
        self._cur_map_to_odom = None

        self._publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=1)
        self._tf_broadcaster = tf.TransformBroadcaster() if self.broadcast_tf else None

        rospy.Subscriber(self.input_odom_topic, Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber(self.map_to_odom_topic, Odometry, self._map_to_odom_callback, queue_size=1)

    @staticmethod
    def _pose_to_mat(odom_msg):
        return np.matmul(
            tf.listener.xyz_to_mat44(odom_msg.pose.pose.position),
            tf.listener.xyzw_to_mat44(odom_msg.pose.pose.orientation),
        )

    @staticmethod
    def _vector3_to_np(vector_msg):
        return np.array([vector_msg.x, vector_msg.y, vector_msg.z], dtype=np.float64)

    @staticmethod
    def _np_to_vector3(vector_np):
        return Vector3(*vector_np.tolist())

    def _build_body_to_base_mat(self):
        translation = [
            rospy.get_param('~body_to_base_x', 0.0),
            rospy.get_param('~body_to_base_y', -0.2),
            rospy.get_param('~body_to_base_z', 0.0),
        ]
        rpy = [
            rospy.get_param('~body_to_base_roll', 0.0),
            rospy.get_param('~body_to_base_pitch', 0.471239),
            rospy.get_param('~body_to_base_yaw', 0.0),
        ]
        body_to_base_mat = tf.transformations.euler_matrix(*rpy)
        body_to_base_mat[0:3, 3] = translation
        rospy.loginfo(
            'LocalizationManager body->base_link mount: xyz=(%.3f, %.3f, %.3f), rpy=(%.6f, %.6f, %.6f)',
            translation[0], translation[1], translation[2], rpy[0], rpy[1], rpy[2],
        )
        return body_to_base_mat

    def _transform_twist_to_base(self, twist_with_covariance):
        transformed_twist = copy.deepcopy(twist_with_covariance)
        rotation_body_to_base = self.body_to_base_mat[0:3, 0:3]
        rotation_base_from_body = rotation_body_to_base.T
        translation_body_to_base = self.body_to_base_mat[0:3, 3]

        angular_body = self._vector3_to_np(twist_with_covariance.twist.angular)
        linear_body = self._vector3_to_np(twist_with_covariance.twist.linear)

        angular_base = np.matmul(rotation_base_from_body, angular_body)
        linear_base = np.matmul(
            rotation_base_from_body,
            linear_body + np.cross(angular_body, translation_body_to_base),
        )

        transformed_twist.twist = Twist(
            linear=self._np_to_vector3(linear_base),
            angular=self._np_to_vector3(angular_base),
        )
        return transformed_twist

    def _odom_callback(self, odom_msg):
        with self._lock:
            self._cur_odom_to_base = odom_msg

    def _map_to_odom_callback(self, odom_msg):
        with self._lock:
            self._cur_map_to_odom = odom_msg

    def _get_snapshot(self):
        with self._lock:
            cur_odom = copy.copy(self._cur_odom_to_base)
            cur_map_to_odom = copy.copy(self._cur_map_to_odom)
        return cur_odom, cur_map_to_odom

    def _publish_tf(self, transform_mat):
        if self._tf_broadcaster is None:
            return
        self._tf_broadcaster.sendTransform(
            tf.transformations.translation_from_matrix(transform_mat),
            tf.transformations.quaternion_from_matrix(transform_mat),
            rospy.Time.now(),
            'camera_init',
            self.output_frame_id,
        )

    def _build_state_estimation(self, cur_odom, cur_map_to_odom):
        if cur_odom is None:
            return None

        if cur_map_to_odom is not None:
            map_to_odom_mat = self._pose_to_mat(cur_map_to_odom)
        else:
            map_to_odom_mat = np.eye(4)

        odom_to_base_mat = self._pose_to_mat(cur_odom)
        map_to_base_mat = np.matmul(
            np.matmul(map_to_odom_mat, odom_to_base_mat),
            self.body_to_base_mat,
        )

        xyz = tf.transformations.translation_from_matrix(map_to_base_mat)
        quat = tf.transformations.quaternion_from_matrix(map_to_base_mat)

        state_estimation = Odometry()
        state_estimation.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        state_estimation.twist = self._transform_twist_to_base(cur_odom.twist)
        state_estimation.header.stamp = cur_odom.header.stamp
        state_estimation.header.frame_id = self.output_frame_id
        state_estimation.child_frame_id = self.output_child_frame_id
        state_estimation.pose.covariance = cur_odom.pose.covariance
        state_estimation.twist.covariance = cur_odom.twist.covariance

        return state_estimation, map_to_odom_mat

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            cur_odom, cur_map_to_odom = self._get_snapshot()
            result = self._build_state_estimation(cur_odom, cur_map_to_odom)
            if result is not None:
                state_estimation, map_to_odom_mat = result
                self._publish_tf(map_to_odom_mat)
                self._publisher.publish(state_estimation)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('localization_manager')
    rospy.loginfo('Localization Manager Node Inited...')
    LocalizationManager().spin()
