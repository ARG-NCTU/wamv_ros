#!/usr/bin/env python
'''
Author: Arthur Hsieh
Date: 2018/08/07
Last update: 2018/08/07                                                             
Point Cloud Clustering
Subscribe: 
  /obj_list				(robotx_msgs/ObjectPoseList)
  /odometry/filtered			(nav_msgs/Odometry)
Publish:
  /wp_line				(visualization_msgs/Marker)
'''
import sys
import rospy
import copy
import numpy as np
from math import sqrt, acos

from tf import TransformListener, TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from robotx_msgs.msg import ObjectPose
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint
from robotx_msgs.msg import WaypointList
from robotx_msgs.srv import waypoint
from std_srvs.srv import *


class point():
	#initialize
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0


class task1_node():
	#initialize
	def __init__(self):
		self.odom_sub		= rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
		self.obj_sub		= rospy.Subscriber("/obj_list/classify", ObjectPoseList, self.object_cb)
		self.pub_rviz		= rospy.Publisher("/wp_line", Marker, queue_size = 1)

		self.listener 		= TransformListener()
		self.transformer 	= TransformerROS()
		self.waypoint_list 	= WaypointList()
		self.position 		= point()
		self.init_entrance	= point()
		self.init_exit		= point()
		self.init_center	= point()
		self.last_entrance	= point()
		self.last_exit		= point()
		self.first_entrance = point()
		self.first_exit		= point()

		# self.Entrance 	= True
		# self.Exit 		= False
		self.SecondObserve	= False
		self.arriveMiddle 	= False
		self.Finish 		= False

		self.length = 30


	def add_waypoint(self, x, y):
		rospy.wait_for_service("/add_waypoint")
		try:
			send_waypoint = rospy.ServiceProxy("/add_waypoint", waypoint)
			waypoint_len = send_waypoint(x, y, 0, 0)

			#return waypoint_len
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def clear_waypoint(self):
		rospy.wait_for_service("/clear_waypoints")
		try:
			clear_waypoint = rospy.ServiceProxy("/clear_waypoints", Trigger)
			waypoint_len = clear_waypoint()

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	#Check if the boat pass the center of the rec
	def arrive_middle(self, pos_x, pos_y, center_x, center_y):
		check_x = pos_x - center_x
		check_y = pos_y - center_y

		if (sqrt(check_x**2 + check_y**2) <= 5):
			self.arriveMiddle = True
		else:
			self.arriveMiddle = False


	#Recalculate the exit point after arrving the middle point
	def exit_after_mid(self, totem_list, entrance, pre_exit):
		pro_exit = point()
		exit_1 = point()
		exit_2 = point()

		dis_list = []
		for i in range(totem_list.size):
			dis_tuple = (i, sqrt((totem_list.list[i].position.x-pre_exit.x)**2+(totem_list.list[i].position.y-pre_exit.y)**2)) 
			dis_list.append(dis_tuple)

		dic = {key: value for (key, value) in dis_list}
		dic = sorted(dic.items(), key=lambda x: x[1])

		exit_1.x = totem_list.list[dic[0][0]].position.x
		exit_1.y = totem_list.list[dic[0][0]].position.y
		exit_2.x = totem_list.list[dic[1][0]].position.x
		exit_2.y = totem_list.list[dic[1][0]].position.y

		pro_exit.x = (exit_1.x + exit_2.x) / 2
		pro_exit.y = (exit_1.y + exit_2.y) / 2

		pro_exit.x = pro_exit.x + (pro_exit.x-entrance.x)/6
		pro_exit.y = pro_exit.y + (pro_exit.y-entrance.y)/6

		return pro_exit


	def task_finish(self):
		print("Finish adding waypoints for task1")


	#Draw the waypoint in rviz
	def rviz(self):
		marker = Marker()
		marker.header.frame_id = "odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.a = 1.0
		marker.color.r = 0
		marker.color.g = 1.0
		marker.color.b = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.points = []

		for i in range(self.waypoint_list.size):
			wp = Point()
			wp.x = self.waypoint_list.list[i].x
			wp.y = self.waypoint_list.list[i].y
			wp.z = 0
			marker.points.append(wp)

		self.pub_rviz.publish(marker)


	#odometry callback functioon
	def odom_cb(self, msg):
		self.position.x = msg.pose.pose.position.x
		self.position.y = msg.pose.pose.position.y
		self.position.z = 0


	#object callback function
	def object_cb(self, msg):
		print("Receive callback")

		self.obj_list = msg

		entrance 	= point()		#middle point of two entrance buoys
		exit 		= point()		#middle point of two exit buoys
		center 		= point()		#center of the rec

		entrance_wp = Waypoint()				
		exit_wp 	= Waypoint()

		obj_size = self.obj_list.size
		marked_index = []


		if self.Finish:
			self.task_finish()
		else:

			#Remove specific area (area that is behind the boat)
			for i in range(self.obj_list.size):
				if (self.obj_list.list[i].position.y < 0 or self.obj_list.list[i].position.y > 20):
					marked_index.append(i)

			for i in sorted(marked_index, reverse=True):
				del self.obj_list.list[i]
				self.obj_list.size = self.obj_list.size - 1
				obj_size = obj_size - 1

			###################### Transform to world coordinate ######################
			try:
				(trans,rot) = self.listener.lookupTransform('/odom', '/velodyne', rospy.Time(0))
				transpose_matrix = self.transformer.fromTranslationRotation(trans,rot)
				#self.robot_pose = np.dot(transpose_matrix, [0, 0, 0, 1])

				for obj_index in range(0, self.obj_list.size):
					center_x = self.obj_list.list[obj_index].position.x
					center_y = self.obj_list.list[obj_index].position.y
					center_z = self.obj_list.list[obj_index].position.z
					centroid = np.array([center_x, center_y, center_z, 1])
					new_center = np.dot(transpose_matrix, centroid)
					self.obj_list.list[obj_index].position.x = new_center[0]
					self.obj_list.list[obj_index].position.y = new_center[1]
					self.obj_list.list[obj_index].position.z = new_center[2]
					self.obj_list.header.frame_id = "odom"

					#Remove specific area (area that is behind the boat)
					#if (self.obj_list.list[obj_index].position.x < self.position.x):
					#	marked_index.append(obj_index)
					#	obj_size = obj_size - 1

				tfSuccess = True

			except (LookupException, ConnectivityException, ExtrapolationException):
				tfSuccess = False
				print("TF receive error")

			###########################################################################
			#Remove specific range
			#for i in sorted(marked_index, reverse=True):
			#	del self.obj_list.list[i]
			#	self.obj_list.size = self.obj_list.size - 1

			#Create a list which contains totem only
			self.totem_list = copy.deepcopy(self.obj_list)

			numberOfTotem = obj_size	#obj_size 	#number of detected totoms
			
			for i in range(obj_size-1, -1, -1):
				if (self.totem_list.list[i].type != "totem"):
					del self.totem_list.list[i]
					self.totem_list.size -= 1
					numberOfTotem -= 1

			if tfSuccess:
				if (self.SecondObserve==False):
					numberOfTotem = 2

				#Four totems are detected
				if (numberOfTotem == 4):
					print("Four totems are detected")
					#Grouping points into entrance and exit group
					for i in range(0, numberOfTotem):
						# if self.arriveMiddle:
						# 	entrance.x = (self.totem_list.list[2].position.x + self.totem_list.list[3].position.x) / 2
						# 	entrance.y = (self.totem_list.list[2].position.y + self.totem_list.list[3].position.y) / 2
						# 	exit.x = (self.totem_list.list[0].position.x + self.totem_list.list[1].position.x) / 2
						# 	exit.y = (self.totem_list.list[0].position.y + self.totem_list.list[1].position.y) / 2
						# else:
						entrance.x = (self.totem_list.list[0].position.x + self.totem_list.list[1].position.x) / 2
						entrance.y = (self.totem_list.list[0].position.y + self.totem_list.list[1].position.y) / 2
						exit.x = (self.totem_list.list[2].position.x + self.totem_list.list[3].position.x) / 2
						exit.y = (self.totem_list.list[2].position.y + self.totem_list.list[3].position.y) / 2

				#Three totems are detected
				elif (numberOfTotem == 3):
					print("Three totems are detected")

					distance1 = sqrt((self.totem_list.list[2].position.x-self.totem_list.list[0].position.x)**2 + (self.totem_list.list[2].position.y-self.totem_list.list[0].position.y)**2)
					distance2 = sqrt((self.totem_list.list[2].position.x-self.totem_list.list[1].position.x)**2 + (self.totem_list.list[2].position.y-self.totem_list.list[1].position.y)**2)

					if (distance1 > distance2):
						x = self.totem_list.list[0].position.x + (self.totem_list.list[2].position.x-self.totem_list.list[1].position.x)
						y = self.totem_list.list[0].position.y + (self.totem_list.list[2].position.y-self.totem_list.list[1].position.y)
					else:
						x = self.totem_list.list[1].position.x + (self.totem_list.list[2].position.x-self.totem_list.list[0].position.x)
						y = self.totem_list.list[1].position.y + (self.totem_list.list[2].position.y-self.totem_list.list[0].position.y)

					entrance.x = (self.totem_list.list[0].position.x + self.totem_list.list[1].position.x) / 2
					entrance.y = (self.totem_list.list[0].position.y + self.totem_list.list[1].position.y) / 2
					exit.x = (self.totem_list.list[2].position.x + x) / 2
					exit.y = (self.totem_list.list[2].position.y + y) / 2


				#Two totems are detected
				elif (numberOfTotem == 2):
					print("Two totems are detected")

					#Check if the boat pass the entrance point
					if(self.SecondObserve and ((self.position.x>0 and (self.position.x-self.last_entrance.x)>0) or (self.position.x<0 and (self.position.x-self.last_entrance.x)<0))):
						entrance.x = self.last_entrance.x
						entrance.y = self.last_entrance.y
						exit.x = self.last_exit.x
						exit.y = self.last_exit.y

					else:
						right_first_exit 	= point()
						right_second_exit 	= point()
						left_first_exit 	= point()
						left_second_exit 	= point()

						#Predict the remaining two totems positions
						#Calculate needed parameters
						m = (self.totem_list.list[0].position.y - self.totem_list.list[1].position.y) /  (self.totem_list.list[0].position.x - self.totem_list.list[1].position.x)
						mPrime = -1 / m
						x = sqrt(self.length**2 / (mPrime**2 + 1))
						y = x * mPrime

						#Right side
						right_first_exit.x = self.totem_list.list[0].position.x + x
						right_first_exit.y = self.totem_list.list[0].position.y + y
						right_second_exit.x = self.totem_list.list[1].position.x + x
						right_second_exit.y = self.totem_list.list[1].position.y + y

						#Left side
						left_first_exit.x = self.totem_list.list[0].position.x - x
						left_first_exit.y = self.totem_list.list[0].position.y - y
						left_second_exit.x = self.totem_list.list[1].position.x - x
						left_second_exit.y = self.totem_list.list[1].position.y - y

						#Check the predicted points with objects in obj_list
						predicted_point = 0
						#for i in range(0, self.obj_list.size):
						#	if (abs(self.obj_list.list[i].position.x-right_first_exit.x)<3 and abs(self.obj_list.list[i].position.y-right_first_exit.y)<3):
						#		predicted_point+=1
						#		Right = True
						#	elif (abs(self.obj_list.list[i].position.x-right_second_exit.x)<3 and abs(self.obj_list.list[i].position.y-right_second_exit.y)<3):
						#		predicted_point+=1
						#		Right = True
						#	elif (abs(self.obj_list.list[i].position.x-left_first_exit.x)<3 and abs(self.obj_list.list[i].position.y-left_first_exit.y)<3):
						#		predicted_point+=1
						#		Right = False
						#	elif (abs(self.obj_list.list[i].position.x-left_second_exit.x)<3 and abs(self.obj_list.list[i].position.y-left_second_exit.y)<3):
						#		predicted_point+=1
						#		Right = False

						#If no predicted totem position
						if (predicted_point == 0):
							sumOfDistance1 = ((right_first_exit.x-self.position.x)**2 + (right_first_exit.y-self.position.y)**2) + ((right_second_exit.x-self.position.x)**2 + (right_second_exit.y-self.position.y)**2)
							sumOfDistance2 = ((left_first_exit.x-self.position.x)**2 + (left_first_exit.y-self.position.y)**2) + ((left_second_exit.x-self.position.x)**2 + (left_second_exit.y-self.position.y)**2)

							if(sumOfDistance1>=sumOfDistance2):
								exit.x = (right_first_exit.x + right_second_exit.x) / 2
								exit.y = (right_first_exit.y + right_second_exit.y) / 2
							else:
								exit.x = (left_first_exit.x + left_second_exit.x) / 2
								exit.y = (left_first_exit.y + left_second_exit.y) / 2

						#If one or more exit points are predicted
						else:
							if Right:
								exit.x = (right_first_exit.x + right_second_exit.x) / 2
								exit.y = (right_first_exit.y + right_second_exit.y) / 2
							else:
								exit.x = (left_first_exit.x + left_second_exit.x) / 2
								exit.y = (left_first_exit.y + left_second_exit.y) / 2

						entrance.x = (self.totem_list.list[0].position.x + self.totem_list.list[1].position.x) / 2
						entrance.y = (self.totem_list.list[0].position.y + self.totem_list.list[1].position.y) / 2

						if (self.SecondObserve==False):
							self.last_entrance.x = entrance.x
							self.last_entrance.y = entrance.y
							self.last_exit.x = exit.x
							self.last_exit.y = exit.y

						self.SecondObserve = True

				else:
					entrance.x = self.last_entrance.x
					entrance.y = self.last_entrance.y
					exit.x     = self.last_exit.x
					exit.y     = self.last_exit.y


				entrance_del = sqrt((entrance.x-self.last_entrance.x)**2 + (entrance.y-self.last_entrance.y)**2)
				exit_del 	 = sqrt((exit.x-self.last_exit.x)**2 + (exit.y-self.last_exit.y)**2)

				if (entrance_del>3 or exit_del>3):
					self.last_entrance.x = entrance.x
					self.last_entrance.y = entrance.y
					self.last_exit.x = exit.x
					self.last_exit.y = exit.y
					
				else:
					entrance.x = self.last_entrance.x
					entrance.y = self.last_entrance.y
					exit.x = self.last_exit.x
					exit.y = self.last_exit.y
				
				# #Double check the entrance and exit points
				# entrance_del = sqrt((entrance.x-self.first_entrance.x)**2 + (entrance.y-self.first_entrance.y)**2)
				# exit_del 	 = sqrt((exit.x-self.first_exit.x)**2 + (exit.y-self.first_exit.y)**2)
				# if (entrance_del > 3 or exit_del > 3):
				# 	entrance.x = self.first_entrance.x
				# 	entrance.y = self.first_entrance.y
				# 	exit.x = self.first_exit.x
				# 	exit.y = self.first_exit.y


				#Calculate waypoints
				entrance_wp.x = entrance.x - (exit.x - entrance.x) / 50
				entrance_wp.y = entrance.y - (exit.y - entrance.y) / 50
				exit_wp.x = exit.x + (exit.x - entrance.x) / 50
				exit_wp.y = exit.y + (exit.y - entrance.y) / 50

				#Calculate center of the rec area
				center.x = (self.last_entrance.x + self.last_exit.x) / 2
				center.y = (self.last_entrance.y + self.last_exit.y) / 2
				#Check if the boat pass the center point
				self.arrive_middle(self.position.x, self.position.y, center.x, center.y)

				################## Add waypoints and call waypoint service ##################
				'''
				Entrance points, Middle points,  Exit points are added to waypoint list sequentially
				1. If it is the same point, then ignore
				2. If it is the different point, add to the waypoint list
				'''
				distance = sqrt((self.position.x-entrance_wp.x)**2 + (self.position.y-entrance_wp.y)**2)
				entrance_thres = sqrt((self.init_entrance.x-entrance_wp.x)**2 + (self.init_entrance.y-entrance_wp.y)**2)
				exit_thres = sqrt((self.init_exit.x-exit_wp.x)**2 + (self.init_exit.y-exit_wp.y)**2)
				center_thres = sqrt((self.init_center.x-center.x)**2 + (self.init_center.y-center.y)**2)
				print entrance_thres
				if (entrance_thres < 5):
					if (center_thres < 5):

						if (exit_thres < 5):
								print("Same waypoints will be ignored")
						elif self.arriveMiddle:
							print("add exit point")
							print(center.x, center.y)
							print(exit_wp.x, exit_wp.y)
							exit_wp = self.exit_after_mid(self.totem_list, entrance_wp, exit_wp)
							self.add_waypoint(exit_wp.x, exit_wp.y)
							self.waypoint_list.list.append(exit_wp)
							self.waypoint_list.size += 1

							self.init_exit.x 	= exit_wp.x
							self.init_exit.y 	= exit_wp.y
							self.Finish 		= True
						else:
							print(entrance_wp.x,entrance_wp.y) 
							print("Still on the way to middle point")
					else:
						print("add center point")
						self.add_waypoint(center.x, center.y)
						self.waypoint_list.list.append(center)
						self.waypoint_list.size += 1

						self.init_center.x 		= center.x
						self.init_center.y 		= center.y
				else:
					print("add entrance point")
					self.add_waypoint(entrance_wp.x, entrance_wp.y)
					self.waypoint_list.list.append(entrance_wp)
					self.waypoint_list.size += 1

					self.init_entrance.x 	= entrance_wp.x
					self.init_entrance.y 	= entrance_wp.y
					

				self.rviz()

				#############################################################################



def main(args):
	rospy.init_node('task1_node', anonymous = True) 
	ic = task1_node()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)