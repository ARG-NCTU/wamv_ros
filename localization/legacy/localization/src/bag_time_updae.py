#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2

class BagTimeUpdate():
	def __init__(self):
		self.origin_x = -1
		self.origin_y = -1
		
		self.nav_x = 0
		self.nav_y = 0

		self.pub_gps = rospy.Publisher("/fix", NavSatFix, queue_size=1)
		self.pub_imu = rospy.Publisher("/imu/data", Imu, queue_size=1)
		self.pub_lidar = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=1)
	
		self.sub_gps = rospy.Subscriber('/fix_', NavSatFix, self.cbfix)
		self.sub_imu = rospy.Subscriber('/imu/data_', Imu, self.cbimu)
		self.sub_lidar = rospy.Subscriber('/velodyne_points_', PointCloud2, self.cblidar)


	def cbfix(self, msg):
		msg.header.stamp = rospy.Time.now()
		self.pub_gps.publish(msg)

	def cbimu(self, msg):
		msg.header.stamp = rospy.Time.now()
		self.pub_imu.publish(msg)

	def cblidar(self, msg):
		msg.header.stamp = rospy.Time.now()
		self.pub_lidar.publish(msg)


if __name__ == '__main__':
	rospy.init_node('bag_time_update', anonymous = True)
	bag_time_update = BagTimeUpdate()
	rospy.spin()
