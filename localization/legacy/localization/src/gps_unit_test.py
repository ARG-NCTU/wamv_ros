#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
import math
import numpy as np
import random
import tf
import matplotlib.pyplot as plt
from scipy.stats import norm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from sensor_msgs.msg import NavSatFix
from geodesy.utm import UTMPoint, fromLatLong

class GPSUnitTest():
	def __init__(self):
		self.origin_x = -1
		self.origin_y = -1
		
		self.nav_x = 0
		self.nav_y = 0

		sub_gps = rospy.Subscriber('/fix', NavSatFix, self.cbfix)
	
	def cbfix(self, msg):
		utm_point = fromLatLong(msg.latitude, msg.longitude)
		if self.origin_x is -1 and self.origin_y is -1:
			self.origin_x = utm_point.easting
			self.origin_y = utm_point.northing
			print "Set origin utm point, x = ", str(self.origin_x), " y = ", str(self.origin_y)
	
		self.nav_x = utm_point.easting - self.origin_x
		self.nav_y = utm_point.northing - self.origin_y
		dis = math.sqrt(self.nav_x*self.nav_x + self.nav_y*self.nav_y)
		print "X = " + str(self.nav_x) + ", Y = " + str(self.nav_y) + \
			", Distance = " + str(dis)


if __name__ == '__main__':
	rospy.init_node('gps_unit_node', anonymous = True)
	gps_unit = GPSUnitTest()
	rospy.spin()
