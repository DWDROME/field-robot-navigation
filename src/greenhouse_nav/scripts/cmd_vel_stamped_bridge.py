#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import rospy
from geometry_msgs.msg import Twist, TwistStamped


DEFAULT_INPUT_TOPIC = '/cmd_vel_stamped'
DEFAULT_OUTPUT_TOPIC = '/cmd_vel'


class CmdVelStampedBridge(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', DEFAULT_INPUT_TOPIC)
        self.output_topic = rospy.get_param('~output_topic', DEFAULT_OUTPUT_TOPIC)

        self._publisher = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        rospy.Subscriber(self.input_topic, TwistStamped, self._callback, queue_size=10)

    def _callback(self, msg):
        self._publisher.publish(msg.twist)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_stamped_bridge')
    rospy.loginfo('CmdVelStampedBridge Node Inited...')
    CmdVelStampedBridge()
    rospy.spin()
