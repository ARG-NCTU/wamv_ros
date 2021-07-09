#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped

def cb_vel(msg):
	vel = TwistWithCovarianceStamped()
	vel.header = msg.header
	vel.twist.twist.linear.x = msg.vector.x
	vel.twist.twist.linear.y = msg.vector.y
	vel.twist.twist.linear.z = msg.vector.z
	vel.twist.covariance[0] = 5.0
	vel.twist.covariance[7] = 5.0
	vel.twist.covariance[14] = 5.0
	pub_vel.publish(vel)

if __name__ == "__main__":
	rospy.init_node("vel_remap_node")
	rospy.Subscriber("/fix_velocity", Vector3Stamped, cb_vel)
	pub_vel = rospy.Publisher("/vel", TwistWithCovarianceStamped, queue_size = 20)
	rospy.spin()
