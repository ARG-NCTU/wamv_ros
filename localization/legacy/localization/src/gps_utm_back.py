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
from math import atan2, cos, sin
from robotx_msgs.srv import SetFloat, SetFloatResponse, SetFloatRequest

class gps_kalman_node():
	def __init__(self):
		self.br = tf.TransformBroadcaster()

		self.imu_offset = 0
		self.srv_imu_offset = rospy.Service('/imu_offset', SetFloat, self.cb_srv_imu_offest)

		self.origin_x = -1
		self.origin_y = -1
		sub_imu = Subscriber("/imu/data", Imu)
		sub_gps = Subscriber('/fix', NavSatFix)
		self.nav_x = 0
		self.nav_y = 0

		ats = ApproximateTimeSynchronizer((sub_imu, sub_gps),queue_size = 1, slop = 0.1)
		ats.registerCallback(self.call_back)


		self.marker_pub = rospy.Publisher('Odom_marker_kalman', Marker, queue_size = 10)
		self.marker_cov_pub = rospy.Publisher('Odom_marker_cov', Marker, queue_size = 10)
		self.marker_raw_pub = rospy.Publisher('Odom_marker_raw', Marker, queue_size = 10)
		self.pub_odom = rospy.Publisher('/odometry/filtered', Odometry, queue_size = 10)
		self.lock = 0
		self.pose = Pose()
		self.covariance = np.zeros((36,), dtype=float)
		self.started = 0
		self.prior_x = 0
		self.prior_y = 0
		self.prior_z = 0
		self.id = 0
		self.frame_id = "odom"
		self.odom = Odometry()
		self.odom.header.frame_id = "odom"
	
	def cb_srv_imu_offest(self, request): 
		self.imu_offset = request.data
		print ("Set imu offset = " + str(self.imu_offset))
		return SetFloatResponse()

	def cbfix(self, msg):
		
		utm_point = fromLatLong(msg.latitude, msg.longitude)
		if self.origin_x is -1 and self.origin_y is -1:
			self.origin_x = utm_point.easting
			self.origin_y = utm_point.northing
			print "Set origin utm point, x = ", str(self.origin_x), " y = ", str(self.origin_y)
	
		self.nav_x = utm_point.easting - self.origin_x
		self.nav_y = utm_point.northing - self.origin_y
		print "Ship_location,  X = " + str(self.nav_x) + ", Y = " + str(self.nav_y)
		'''

		if self.origin_x is -1 and self.origin_y is -1:
			self.origin_x = msg.longitude
			self.origin_y = msg.latitude
			print "Set origin gps point, x = ", str(self.origin_x), " y = ", str(self.origin_y)

		#self.gpsDistance(msg.latitude, msg.longitude, self.origin_y, self.origin_x)
		self.gpsDistance(self.origin_y, self.origin_x, msg.latitude, msg.longitude)
		print "Ship_location,  X = " + str(self.nav_x) + ", Y = " + str(self.nav_y)
		'''

	def gpsDistance(self, lat1, lon1, lat2, lon2):
		R = 6371000
		phi_1 = math.radians(lat1)
		phi_2 = math.radians(lat2)
		delta_phi = math.radians(lat2 - lat1)
		delta_lambda = math.radians(lon2 - lon1)
		a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
		meters = R * c
		meters = round(meters, 3)

		bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)) + math.pi/2
		bearing = -bearing
		bearing_deg = - math.degrees(bearing)
		bebearing_degaring = (bearing_deg + 360) % 360

		#print "meters = ", meters, ", angles = ", bearing_deg
		self.nav_x = meters * cos(bearing)
		self.nav_y = meters * sin(bearing)


	def call_back(self, odom_imu, odom_gps):
		if self.lock == 0:
			self.lock = 1
			self.cbfix(odom_gps)
			self.pose.position.x = self.nav_x
			self.pose.position.y = self.nav_y
			self.pose.position.z = 0
			q = (odom_imu.orientation.x, odom_imu.orientation.y, odom_imu.orientation.z, odom_imu.orientation.w)
			euler = tf.transformations.euler_from_quaternion(q)
			qu = tf.transformations.quaternion_from_euler(0, 0, euler[2]+self.imu_offset)
			self.pose.orientation.x = qu[0]
			self.pose.orientation.y = qu[1]
			self.pose.orientation.z = qu[2]
			self.pose.orientation.w = qu[3]
			print euler[2]/3.14 * 180	
			#self.covariance = odom_imu.pose.covariance
			self.process()
		
	def process(self):
		if self.started == 0:
			# initialize prior
			self.prior_x = norm(loc = self.pose.position.x, scale = 100)
			self.prior_y = norm(loc = self.pose.position.y, scale = 100)
			self.prior_z = norm(loc = self.pose.position.z, scale = 100)
			self.started = 1
			self.lock = 0
			return
		x = self.pose.position.x
		y = self.pose.position.y
		z = self.pose.position.z
		covariance = self.covariance

		#prediction step
		kernel = norm(loc = 0, scale = 2)
		predicted_x = norm(loc = self.prior_x.mean()+kernel.mean(), scale = np.sqrt(self.prior_x.var()+kernel.var()))
		predicted_y = norm(loc = self.prior_y.mean()+kernel.mean(), scale = np.sqrt(self.prior_y.var()+kernel.var()))
		predicted_z = norm(loc = self.prior_z.mean()+kernel.mean(), scale = np.sqrt(self.prior_z.var()+kernel.var()))
		#update step
		posterior_x = self.update_con(predicted_x, x, 0.05)
		posterior_y = self.update_con(predicted_y, y, 0.05)
		posterior_z = self.update_con(predicted_z, z, 0.05)
		#print covariance[0]
		self.prior_x = posterior_x
		self.prior_y = posterior_y
		self.prior_z = posterior_z

		#print "lat:", (posterior_x.mean()), posterior_x.var()
		#print "long:", (posterior_y.mean()), posterior_y.var()
		#print "alt:", (posterior_z.mean()), posterior_z.var()
		self.odom.pose.pose.position.x = posterior_x.mean()
		self.odom.pose.pose.position.y = posterior_y.mean()
		self.odom.pose.pose.position.z = posterior_z.mean()
		self.odom.pose.pose.orientation = self.pose.orientation
		#print self.odom.pose.pose.orientation
		self.odom.header.stamp = rospy.Time.now()
		self.pub_odom.publish(self.odom)
		
		self.br.sendTransform((self.odom.pose.pose.position.x, \
                         self.odom.pose.pose.position.y, self.odom.pose.pose.position.z), \
                        (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, \
                        self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w), \
                        rospy.Time.now(),"/base_link","/odom")

		'''self.br.sendTransform((0, \
                         0, 0), \
                        (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, \
                        self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w), \
                        rospy.Time.now(),"/base_link","/base_link_fake")'''

		if self.origin_x is not -1:
			q = tf.transformations.quaternion_from_euler(0, 0, 0)
			self.br.sendTransform((self.origin_x, self.origin_y, 0), \
				(q[0], q[1], q[2], q[3]), \
				rospy.Time.now(),"/odom","/utm")					

		self.lock = 0
		## Draw marker
		marker = Marker(type=Marker.SPHERE, \
			id=self.id, lifetime=rospy.Duration(), \
			pose=Pose(Point(float((posterior_x.mean())), float((posterior_y.mean())), float(posterior_z.mean())), \
			Quaternion(0, 0, 0, 1)),\
			scale=Vector3(0.3, 0.3, 0.3),\
			header=Header(frame_id = self.frame_id),\
			color=ColorRGBA(0.0, 0.0, 0.0, 1))
		self.marker_pub.publish(marker)
		
		marker_cov = Marker(type=Marker.SPHERE, \
			id=self.id+10000, lifetime=rospy.Duration(0.1), \
			pose=Pose(Point(float((posterior_x.mean())), float((posterior_y.mean())), float(posterior_z.mean())), \
			Quaternion(0, 0, 0, 1)),\
			#scale=Vector3(np.sqrt(covariance[4]), np.sqrt(covariance[0]), np.sqrt(covariance[8])),\
			scale=Vector3(np.sqrt(posterior_x.var()), np.sqrt(posterior_y.var()), np.sqrt(posterior_z.var())), \
			header=Header(frame_id = self.frame_id),\
			color=ColorRGBA(1, 0.6, 0.8, 0.3))
		self.marker_cov_pub.publish(marker_cov)


		marker_raw = Marker(type=Marker.SPHERE, \
			id=self.id+200000, lifetime=rospy.Duration(), \
			pose=Pose(Point(float((x)), float((y)), float(z)), \
			Quaternion(0, 0, 0, 1)),\
			scale=Vector3(0.3, 0.3, 0.3),\
			header=Header(frame_id = self.frame_id),\
			color=ColorRGBA(1.0, 0.0, 0.0, 1))
		self.marker_raw_pub.publish(marker_raw)
		self.id += 1


	def measurement(self, measurementx, variance):
		likelihood = norm(loc = measurementx, scale = np.sqrt(variance))
		return likelihood

	def gaussian_multiply(self, g1, g2):
		g1_mean, g1_var = g1.stats(moments='mv')
		g2_mean, g2_var = g2.stats(moments='mv')
		mean = (g1_var * g2_mean + g2_var * g1_mean) / (g1_var + g2_var)
		variance = (g1_var * g2_var) / (g1_var + g2_var)
		#print mean, variance
		return norm(loc = mean, scale = np.sqrt(variance))

	def update_con(self, prior, measurementz, covariance):
		likelihood = self.measurement(measurementz, covariance)
		posterior = self.gaussian_multiply(likelihood, prior)
		return posterior

def main():
	ic = gps_kalman_node()
	rospy.init_node('gps_kalman_node', anonymous = True)
	rospy.spin()

if __name__ == '__main__':
	main()
