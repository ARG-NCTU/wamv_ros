#!/usr/bin/env python
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

class task2_node():
	def __init__(self):
		# ROS subscribers and Publishers
		self.obj_sub = rospy.Subscriber("/obj_list/classify", ObjectPoseList, self.object_cb)
		self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
		# class attributes

		self.position.x = 0
		self.position.y = 0
		self.gate_totem_list = []
		self.obj_list = None
		self.state_now = 1

	

	def start_waypoint_nav(self):
		rospy.wait_for_service("/start_")
		try:
			start_waypoint_srv = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
			_ = start_waypoint_srv()

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e 

	def add_waypoint(self, x, y, yaw):
		rospy.wait_for_service("/add_waypoint")
		try:
			send_waypoint_srv = rospy.ServiceProxy("/add_waypoint", waypoint)
			waypoint_len = send_waypoint_srv(x, y, yaw, 0)

			#return waypoint_len
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def clear_waypoint(self):
		rospy.wait_for_service("/clear_waypoints")
		try:
			clear_waypoint_srv = rospy.ServiceProxy("/clear_waypoints", Trigger)
			_ = clear_waypoint_srv()

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def FSM(self, action):
		# Finite State Machine
		'''
		state 1: Idle
		state 2: Search totems and start point
		state 3: Go to start point and listen to pinger
		state 4: Go to aux point through target gate 1 and find target totem(yellow/black)
		state 5: circle target totem, return to aux point and listen to pinger
		state 6: Go to start point through target gate 2
		
		transition 1->2 "start" start
		transition 2->3 "start_p_found" start point found
		transition 3->4 "target_g1_found" target gate 1 found
		transition 4->5 "target_t_found" target totem found
		transition 5->6 "target_g2_found" target gate 2 found
		'''
		if self.state_now == 1 && action == "start":
			self.state_now = 2
		if self.state_now == 2 && action == "start_p_found":
			self.state_now = 3
		if self.state_now == 3 && action == "target_g1_found":
			self.state_now = 4
		if self.state_now == 4 && action == "target_t_found":
			self.state_now = 5
		if self.state_now == 5 && action == "target_g2_found":
			self.state_now = 6
		return 

	def process(self):
		if self.state_now == 1:
			print "wait for start trigger"
		if self.state_now == 2:
			# find start point
			start_waypoint, aux_waypoint = self.search_start_point()		
			FSM("start_p_found")
		if self.state_now == 3:
			# move to start point
			self.add_waypoint(start_waypoint[0], start_waypoint[1], 0)
			self.start_waypoint_nav()
			while(np.sqrt((self.position.x - start_waypoint[0])*(self.position.x - start_waypoint[0]) + (self.position.y - start_waypoint[1])*(self.position.y - start_waypoint[1]))>3):
				rospy.sleep(1):
			# listen to pinger
			gate1_waypoint = self.find_gate(1)
		if self.state_now == 4:
			# go to gate waypoint then aux point
			self.clear_waypoint()
			self.add_waypoint(gate1_waypoint[0], gate1_waypoint[1], 0)
			self.add_waypoint(aux_waypoint[0], aux_waypoint[1], 0)
			self.start_waypoint_nav()
			while(np.sqrt((self.position.x - aux_waypoint[0])*(self.position.x - aux_waypoint[0]) + (self.position.y - aux_waypoint[1])*(self.position.y - aux_waypoint[1]))>3):
				rospy.sleep(1):
			# search for target totem
			totem_waypoint = self.search_target_totem()

		if self.state_now == 5:
			# circle totem
			self.clear_waypoint()
			self.add_waypoint(totem_waypoint[0], totem_waypoint[1]-5, 0)
			self.add_waypoint(totem_waypoint[0]+5, totem_waypoint[1], 0)
			self.add_waypoint(totem_waypoint[0], totem_waypoint[1]+5, 0)
			self.add_waypoint(totem_waypoint[0]-5, totem_waypoint[1], 0)
			self.add_waypoint(totem_waypoint[0], totem_waypoint[1]+5, 0)
			self.add_waypoint(aux_waypoint[0], aux_waypoint[1], -3.14)
			self.start_waypoint_nav()
			rospy.sleep(10):
			while(np.sqrt((self.position.x - aux_waypoint[0])*(self.position.x - aux_waypoint[0]) + (self.position.y - aux_waypoint[1])*(self.position.y - aux_waypoint[1]))>3):
				rospy.sleep(1):
			# listen to pinger
			gate2_waypoint = self.find_gate(2)
		if self.state_now == 6:
			# go to gate waypoint then start point
			self.clear_waypoint()
			self.add_waypoint(gate2_waypoint[0], gate2_waypoint[1], 0)
			self.add_waypoint(start_waypoint[0], start_waypoint[1], 0)
			self.start_waypoint_nav()
			while(np.sqrt((self.position.x - start_waypoint[0])*(self.position.x - start_waypoint[0]) + (self.position.y - start_waypoint[1])*(self.position.y - start_waypoint[1]))>3):
				rospy.sleep(1):




	def search_start_point(self):
		# retrieve obj_list
		# find red white white green totems sequence
		for i in range(self.obj_list.size):
			if self.obj_list.list[i].type == "red_totem" or self.obj_list.list[i].type == "white_totem" or self.obj_list.list[i].type == "green_totem":
				self.gate_totem_list.append(self.obj_list.list[i])

		# check if there are unneccessary totems
		red_totem_count = 0
		green_totem_count = 0
		white_totem_count = 0
		for i in range(len(self.gate_totem_list)):
			if self.gate_totem_list[i].type == "red_totem":
				red_totem_count = red_totem_count + 1
			if self.gate_totem_list[i].type == "green_totem":
				green_totem_count = red_totem_count + 1
			if self.gate_totem_list[i].type == "white_totem":
				white_totem_count = red_totem_count + 1

		pos_x = []
		pos_y = []
		error_margin = 3
		if len(self.gate_totem_list) == 4:
			if white_totem_count == 2:
				if red_totem_count == 1 and green_totem_count == 1:
					for i in range(len(self.gate_totem_list)):
						# linear regression
						pos_x.append(self.gate_totem_list[i].position.x)
						pos_y.append(self.gate_totem_list[i].position.y)
					slope, intercept, residual, _, _ = np.polyfit(pos_x, pos_y, 1, full = True)
					center_x = sum
					pos_x = []
					pos_y = []
					if residual > error_margin:
						for i in range(len(self.gate_totem_list)):
						if self.gate_totem_list[i].type == "white_totem":
							pos_x.append(self.gate_totem_list[i].position.x)
							pos_y.append(self.gate_totem_list[i].position.y)
						slope, intercept, residual, _, _ = np.polyfit(pos_x, pos_y, 1, full = True)
						center_x = sum(pos_x)/4
						center_y = sum(pos_y)/4
				else:
					pos_x = []
					pos_y = []
					for i in range(len(self.gate_totem_list)):
					if self.gate_totem_list[i].type == "white_totem":
						pos_x.append(self.gate_totem_list[i].position.x)
						pos_y.append(self.gate_totem_list[i].position.y)
					slope, intercept, residual, _, _ = np.polyfit(pos_x, pos_y, 1, full = True)
					center_x = sum(pos_x)/2
					center_y = sum(pos_y)/2
			# TODO less than two white totems detected
		else:
			pos_x = []
			pos_y = []
			if white_totem_count == 2:
				for i in range(len(self.gate_totem_list)):
				if self.gate_totem_list[i].type == "white_totem":
					pos_x.append(self.gate_totem_list[i].position.x)
					pos_y.append(self.gate_totem_list[i].position.y)
				slope, intercept, residual, _, _ = np.polyfit(pos_x, pos_y, 1, full = True)
				center_x = sum(pos_x)/2
				center_y = sum(pos_y)/2
			# TODO less than two white totems detected		


		# Calculate start point and aux point
		vec_x = slope/np.sqrt(slope*slope+1)
		vec_y = -1/np.sqrt(slope*slope+1)
		hypo1_x = center_x + 10 * vec_x
		hypo1_y = center_y + 10 * vec_y
		hypo2_x = center_x + 10 * vec_x
		hypo2_y = center_y + 10 * vec_y


		dist_to_hypo1 = np.sqrt((self.position.x - hypo1_x)*(self.position.x - hypo1_x) + (self.position.y - hypo1_y)*(self.position.y - hypo1_y))
		dist_to_hypo2 = np.sqrt((self.position.x - hypo2_x)*(self.position.x - hypo2_x) + (self.position.y - hypo2_y)*(self.position.y - hypo2_y))
					
		if dist_to_hypo1 < dist_to_hypo2:
			start_waypoint = np.array([hypo1_x, hypo1_y])
			aux_waypoint = np.array([hypo2_x, hypo2_y])
		else:
			aux_waypoint = np.array([hypo1_x, hypo1_y])
			start_waypoint = np.array([hypo2_x, hypo2_y])
	
		return start_waypoint, aux_waypoint

	def find_gate(self, num):
		# retrieve pinger_data

		if target_gate_found:
			if num == 1:
				FSM("target_g1_found")
			else:
				FSM("target_g2_found")
		return gate_waypoint

	def search_target_totem(self):
		for i in range(self.obj_list.size):
			if self.obj_list.list[i].type == "yellow_totem":
				FSM("target_t_found")
				return np.array([self.obj_list.list[i].position.x, position.y])

	def object_cb(self, msg):
		self.obj_list = msg

	def odom_cb(self, msg):
		self.position.x = msg.pose.pose.position.x
		self.position.y = msg.pose.pose.position.y




def main(args):
	rospy.init_node('task2_node', anonymous = True)
	ic = task2_node()
	try:
		while(1):
			ic.process()
			rospy.sleep(0.1)
	except KeyboardInterrupt:
		print "shutdown"
		## Need to do ros node cleanup


if __name__ == '__main__':
	main(sys.argv)