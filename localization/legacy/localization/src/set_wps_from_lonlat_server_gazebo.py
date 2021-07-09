#!/usr/bin/env python

# Changelog:
#	Date: 8/2
# 	Since we decided to use East as x-axis direction, theta_ is equal to
#	magnetic declination given in wam_v_navsat.yaml, which is almost zero.
#	So I ignored theta_ variable

import rospy
import utm
import tf
from math import cos, sin
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from localization.srv import *

pose_array = PoseArray()
pose_array.header.frame_id = 'odom'
status = False
x_ = y_ = None

def handle_set_wps(req):
	global x_, y_
	global status
	global pose_array
	global pub_wps
	
	utm_x, utm_y, _, _ = utm.from_latlon(req.lat, req.lon)
	
	resp = set_wps_from_lonlatResponse()
	
	if status == True:
		resp.x = utm_x - x_
		resp.y = utm_y - y_
		rospy.loginfo("[%s] odom x: %s, y: %s" %(rospy.get_name(), resp.x, resp.y))
		p = Pose()
		p.position.x = resp.x
		p.position.y = resp.y
		p.orientation.w = 1
		pose_array.poses.append(p)
		pub_wps.publish(pose_array)
		return resp

# For Gazebo usage
def cb_tf():
	listener = tf.TransformListener()
	global x_, y_
	global status
	if status == True:
		return
	try:
		listener.waitForTransform("/utm", "/odom", rospy.Time(), rospy.Duration(5.0))
		(trans, rot) = listener.lookupTransform("/utm", "/odom", rospy.Time(0))
		x_ = trans[0]
		y_ = trans[1]
		'''quat_ = [rot[0], rot[1], rot[2], rot[3]]
		euler_ = tf.transformations.euler_from_quaternion(quat_)
		theta_ = euler_[2]'''
		rospy.loginfo("[%s] x: %s, y: %s" %(rospy.get_name(), x_, y_))
		status = True
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "tf exception"

		
def pub_tf(event):
	global pose_array
	wp_len = len(pose_array.poses)
	br = tf.TransformBroadcaster()
	for i in range(wp_len): 
		frame_name = "wp_" + str(i)
		br.sendTransform((pose_array.poses[i].position.x, pose_array.poses[i].position.y, 0.0),
				 (0.0, 0.0, 0.0, 1.0),
				 rospy.Time.now(),
				 frame_name,
				 "odom")
def main():
	global pub_wps, sub_tf
	rospy.init_node('set_wps_from_lonlat_server_node')
	server = rospy.Service('set_wps_from_lonlat', set_wps_from_lonlat, handle_set_wps)
	cb_tf()
	pub_wps = rospy.Publisher('/waypoints_lonlat', PoseArray, queue_size = 20)
	rospy.Timer(rospy.Duration(1.0), pub_tf)
	rospy.spin()


if __name__ == "__main__":
	main()
