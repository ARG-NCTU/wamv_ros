#! /usr/bin/env python

import rospy
from robotx_msgs.msg import roboteq_drive
from std_msgs.msg import Float32


rospy.init_node('real2sim')

pub_r = rospy.Publisher('wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)
pub_l = rospy.Publisher('wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)


def cb_cmd(msg):
    print(msg)
    pub_l.publish(msg.left)
    pub_r.publish(msg.right)


sub = rospy.Subscriber('cmd_drive', roboteq_drive, cb_cmd, queue_size=1)

rospy.spin()