#!/usr/bin/env python

import math

# ROS Lib
import sys
import time
import rospy
import tf
import numpy as np

# ROS Msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32

from robotx_msgs.msg import ObjectPoseList

# David's waypoint control
# from sensor_msgs.msg import CompressedImage, Image
# from robotx_msgs.msg import Waypoint
# from robotx_msgs.srv import waypoint, waypointRequest, waypointResponse


# Daniel's waypoint control
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from robotx_msgs.msg import Waypoint, WaypointList
from robotx_msgs.srv import *

from pinger_angle_estimation.srv import *


# Choice of Direction Of Arrival algorithm
ALGORITHM_DOA = 'tdoa_spl_douFilter'

class task2_node():
	def __init__(self):
		# ros related
		self.odom_sub		= rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
		self.obj_sub		= rospy.Subscriber("/obj_list/map", ObjectPoseList, self.objlist_cb)
		self.sub_waypoint_statud = rospy.Subscriber("/wp_nav_state", Int32, self.wpstatus_cb)
		

		self.x = 0
		self.y = 0
		self.yaw = 0
		self.objlist = None
		self.fsm_state = 0
		self.gate_0 = [0, 0]
		self.gate_1 = [0, 0]
		self.gate_2 = [0, 0]
		self.start_point = [0, 0]
		self.aux_point = [0, 0]
		self.target_gate = 0
		self.target_totem_color = "black" # or "yellow"
		self.target_totem = [0, 0]
		self.status = 1    # Flag for checking whether WAM-V is in station keeping condition
		self.direction_to_gate = 0

		self.equip_dict = {'red_totem': [0, 0],
							'white_totem1': [0, 0],
							'white_totem2': [0, 0],
							'green_totem': [0, 0],
							'black_totem': [0, 0],
							'light_totem': [0, 0],
		}

		# self.flag_check
		# print self.equip_dict

		# rospy.wait_for_service(ALGORITHM_DOA, timeout = 1)
  #       pinger2_handle = rospy.ServiceProxy(ALGORITHM_DOA, AngleEstimation)
  #       resp = pinger2_handle(0)

	def wpstatus_cb(self, msg):
		self.status = msg.data

	def odom_cb(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.yaw = euler[2]
		# print self.x, self.y, self.yaw

	def objlist_cb(self, msg):
		self.objlist = msg

	def fsm_transit(self, state_to_transit):
		self.fsm_state = state_to_transit

	def listen_gate_pinger(self):
		# TODO pinger code
		'''try:
									angle_estimate = rospy.ServiceProxy("/tdoa_spl_douFilter", AngleEstimation)
									angle_srv = AngleEstimationRequest()
									angle_srv.command = 5
									
									res = AngleEstimationResponse()
									# rospy.wait_for_service("/tdoa_spl_douFilter")
									# res.angle = angle_estimate(angle_srv)
									res
									res.angle = 20
									print res.angle
						
									rad = self.yaw + res.angle * np.pi / 180.0
									theta_range = np.zeros(4)
									theta_range[0] =  self.angleP2P(self.equip_dict['red_totem'], [self.x, self.y])
									theta_range[1] = self.angleP2P(self.equip_dict['white_totem1'], [self.x, self.y])
									theta_range[2] = self.angleP2P(self.equip_dict['white_totem2'], [self.x, self.y])
									theta_range[3] = self.angleP2P(self.equip_dict['green_totem'], [self.x, self.y])
									print theta_range / np.pi * 180.0
									# if 
									# print gate 1 2 3 range
						
									# res.angle
						
									rospy.sleep(5)
								except rospy.ServiceException, e:
									print "Task2 service call failed: %s"%e'''


		self.target_gate = 1
		return True

	def distance(self, a, b):
		return np.sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]))

	def avg(self, a, b):
		return [(a[0]+b[0])/2, (a[1]+b[1])/2]

	def angleP2P(self, pos, goal):
		a = 90 - math.degrees(math.atan2(pos[0]-goal[0], pos[1]-goal[1]))
        #if a < 0:
        #    a = a+360
		return np.round(a, 1)

	def add_waypoint(self, x, y, yaw):

		clear_wp = rospy.ServiceProxy("/clear_waypoints", Trigger)
		try:
			rospy.wait_for_service("/clear_waypoints")
			clear_wp()
		except rospy.ServiceException, e:
			print "Task2 service call failed: %s"%e

		try:
			send_waypoint = rospy.ServiceProxy("/add_waypoint", waypoint)
			wp_srv = waypointRequest()
			wp_srv.waypointx = x
			wp_srv.waypointy = y
			wp_srv.yaw = yaw
			
			res = waypointResponse()
			rospy.wait_for_service("/add_waypoint")
			res.waypoint_len = send_waypoint(wp_srv)
			print "Task2: set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw/math.pi*180
			time.sleep(3)

		except rospy.ServiceException, e:
			print "Task2 service call failed: %s"%e

		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()

	def find_start_and_aux_point(self):
		amp = 10

		d = self.distance(self.gate_0, self.gate_1)
		vec = [(self.gate_1[1] - self.gate_0[1])/d, -(self.gate_1[0] - self.gate_0[0])/d]
		candidate1 = [self.gate_1[0]+vec[0]*amp, self.gate_1[1]+vec[1]*amp]
		candidate2 = [self.gate_1[0]-vec[0]*amp, self.gate_1[1]-vec[1]*amp]
		if self.distance([self.x, self.y], candidate1) > self.distance([self.x, self.y], candidate2):
			self.start_point = candidate2
			self.aux_point = candidate1
		else:
			self.start_point = candidate1
			self.aux_point = candidate2

	def find_gate_pos(self):
		red_totem = [0, 0]
		white_totem = []
		green_totem = [0, 0]
		# gate 0: red-white1
		# gate 1: white1-white2
		# gate 2: white2-green

		# find totem via distance 
		totem_list = []
		# LIMIT_RANGE_DISTANCE = 20
		for i in range(len(self.objlist.list)):
			if self.objlist.list[i].type == 'totem':
				d = self.distance([self.x, self.y], [self.objlist.list[i].position.x, self.objlist.list[i].position.y])
				theta = self.angleP2P([self.x, self.y], [self.objlist.list[i].position.x, self.objlist.list[i].position.y])
				totem_list.append([self.objlist.list[i], theta, d])

		# regard 4 'cloest' obj as gate_list
		gate_list = sorted(totem_list, key=lambda totem: totem[2])[0:5] 

		gate_list = sorted(gate_list, key=lambda totem: totem[1]) 

		# for i in range(len(gate_list)):
			# print 'gate,', i, ' : x=', gate_list[i][0].position.x, ', y=', gate_list[i][0].position.y, ', d=', gate_list[i][2], ', theta=', gate_list[i][1]
		
		self.equip_dict['green_totem'] = [gate_list[0][0].position.x, gate_list[0][0].position.y]
		self.equip_dict['white_totem2'] = [gate_list[1][0].position.x, gate_list[1][0].position.y]
		self.equip_dict['white_totem1'] = [gate_list[2][0].position.x, gate_list[2][0].position.y]
		self.equip_dict['red_totem'] = [gate_list[3][0].position.x, gate_list[3][0].position.y]
		self.equip_dict['black_totem'] = [gate_list[4][0].position.x, gate_list[4][0].position.y]

		# print 'boat: x=', self.x, ', y=', self.y, ', yaw=', self.yaw
		# print self.equip_dict


		# print 'find green and red totem'
		# for i in range(self.objlist.size):
		# 	if self.objlist.list[i].type == "totem" and self.objlist.list[i].color == "red":
		# 		# TODO: do error recovery if more than one red totem is detected
		# 		# maybe get the one closest to WAMV?

		# 		red_totem = [self.objlist.list[i].position.x, self.objlist.list[i].position.y] 
			
		# 	elif self.objlist.list[i].type == "totem" and self.objlist.list[i].color == "green":
		# 		green_totem = [self.objlist.list[i].position.x, self.objlist.list[i].position.y]
		# if(red_totem == [0, 0]):
		# 	print 'failed to find red totem...'

		# if(green_totem == [0, 0]):
		# 	print 'failed to find green totem...'

		# for i in range(self.objlist.size):
		# 	if self.objlist.list[i].type == "totem" and self.objlist.list[i].color == "white":
		# 		tmp_pos = [self.objlist.list[i].position.x, self.objlist.list[i].position.y]
		# 		if red_totem != [0, 0] and green_totem != [0, 0]:
		# 			if white_totem is None:
		# 				white_totem = [[self.objlist.list[i].position.x, self.objlist.list[i].position.y]]
		# 			else:
		# 				if self.distance(white_totem[0], red_totem) < self.distance(tmp_pos, red_totem):
		# 					white_totem.append(tmp_pos)
		# 				else:
		# 					white_totem.insert(0, tmp_pos)

		# calculate gate positions
		self.gate_0 = self.avg(self.equip_dict['red_totem'], self.equip_dict['white_totem1'])
		self.gate_1 = self.avg(self.equip_dict['white_totem1'], self.equip_dict['white_totem1'])
		self.gate_2 = self.avg(self.equip_dict['white_totem2'], self.equip_dict['green_totem'])

		# print self.gate_0
		# print self.gate_1
		# print self.gate_2

		# checking if the positions are correct
		if self.equip_dict['red_totem'] == [0, 0] or self.equip_dict['white_totem1'] == [0, 0] or \
			self.equip_dict['white_totem2'] == [0, 0] or self.equip_dict['green_totem'] == [0, 0]:
			print "some totems are not detected"
			time.sleep(2)
			return False

		gate_width_0 = np.abs(self.distance(self.equip_dict['red_totem'], self.equip_dict['white_totem1']))
		gate_width_1 = np.abs(self.distance(self.equip_dict['white_totem1'], self.equip_dict['white_totem2']))
		gate_width_2 = np.abs(self.distance(self.equip_dict['white_totem2'], self.equip_dict['green_totem']))
		gate_width_avg = (gate_width_0 + gate_width_1 + gate_width_2)/3

		# ERROR_GATE_WIDTH = 6
		# if np.abs(gate_width_avg - gate_width_0) > ERROR_GATE_WIDTH or np.abs(gate_width_avg - gate_width_1) > ERROR_GATE_WIDTH or np.abs(gate_width_avg - gate_width_2) > ERROR_GATE_WIDTH:
		# 	print "gate detection error"
		# 	time.sleep(2)
		# 	return False

		# while True:
		# 	time.sleep(2)
		return True

	def get_target_totem(self, totem_color):

		# for i in range(len(self.objlist.list)):
		# 	if self.objlist.list[i].type == 'totem':
		# 		if self.a

		self.target_totem = self.equip_dict['black_totem']
		return True
		# for i in range(self.objlist.size):
		# 	if self.objlist.list[i].color == totem_color:
		# 		self.target_totem  = [self.objlist.list[i].position.x, self.objlist.list[i].position.y]
		# 		return True
		# return False

	def circle_the_totem(self):
		#####################################################
		#	Get the 4 points around of target totem
		#####################################################`
		#print angle/math.pi*180
		radius = 3
		pose_candidate = []
		pose_candidate.append([self.target_totem[0] + radius, self.target_totem[1], -np.pi, self.target_totem[0] + radius*2, self.target_totem[1]])
		pose_candidate.append([self.target_totem[0]  , self.target_totem[1] + radius, -np.pi/2, self.target_totem[0], self.target_totem[1] + radius*2])
		pose_candidate.append([self.target_totem[0] - radius, self.target_totem[1], 0, self.target_totem[0] - radius*2, self.target_totem[1]])
		pose_candidate.append([self.target_totem[0]  , self.target_totem[1] - radius, np.pi/2, self.target_totem[0], self.target_totem[1] - radius*2])
		# pose_list = [pose0, pose1, pose2, pose3]

		for p in range(len(pose_candidate)):
			self.add_waypoint(p[0], p[1], p[3])

		return true
		# find the pose that has minimum difference yaw  
		# min_diff_yaw = 
		# for pos in enumerate(pose_candidate):



		#self.update_closest(pose_list)

	def process(self):
		if self.objlist is None or self.objlist.size is 0:
			time.sleep(1)
			print 'no obj detect'
			return

		if self.fsm_state == 0:
			# get objlist to find gates
			print "finding gates"
			while(self.find_gate_pos() == False):
				pass
			self.fsm_transit(1)

		if self.fsm_state == 1:
			print "find start and aux points"
			while(self.find_start_and_aux_point() == False):
				pass

			# travel to start point
			print "travel to start point"
			self.direction_to_gate = math.atan2(self.gate_1[1] - self.start_point[1], self.gate_1[0] - self.start_point[0])
			self.add_waypoint(self.start_point[0], self.start_point[1], yaw=self.direction_to_gate) # not sure whether yaw is correct
			while self.status is 0:

				pass
			
					
			self.fsm_transit(2)

		if self.fsm_state == 2:
			# listen to pinger
			# while True:
			# 	time.sleep(2)
			print "listening to pingers"
			while(self.listen_gate_pinger() ==  False):
				pass
			self.fsm_transit(3)

		if self.fsm_state == 3:
			
			if self.target_gate == 0:
				gate = self.gate_0
			elif self.target_gate == 1:
				gate = self.gate_1
			elif self.target_gate == 2:
				gate = self.gate_2

			# travel to target gate and aux point
			# gate, self.aux_point
			print "pass through the gate"
			self.add_waypoint(gate[0], gate[1], yaw=self.direction_to_gate) # not sure whether yaw is correct
			while self.status is 0:
				pass

			print "travel to aux point"
			direction_to_aux = math.atan2(self.aux_point[1] - gate[1], self.aux_point[0] - gate[0])
			self.add_waypoint(self.aux_point[0], self.aux_point[1], yaw=direction_to_aux) # not sure whether yaw is correct
			while self.status is 0:
				pass

			self.fsm_transit(4)
		if self.fsm_state == 4:
			# get target totem 
			print "find target totem"
			# while True:
			# 	time.sleep(2)
			while(self.get_target_totem(self.target_totem_color) == False):
				pass 
			self.fsm_transit(5)


		if self.fsm_state == 5:
			# circle the target totem and come back
			while(self.circle_the_totem() == False):
				pass
			# travel to ready pose
			# circle totem once

			self.fsm_transit(6)
		
		if self.fsm_state == 6:
			print "back to aux point"
			direction_to_aux = -direction_to_aux
			self.add_waypoint(self.aux_point[0], self.aux_point[1], yaw=direction_to_aux) # not sure whether yaw is correct
			while self.status is 0:
				pass

			print "listening to pingers"
			while(self.listen_gate_pinger() ==  False):
				pass
			# get objlist to find gates
			# print "finding gates"
			# while(self.find_gate_pos() == False):
			# 	pass 
			self.fsm_transit(7)

		if self.fsm_state == 7:
			while(self.find_start_and_aux_point() == False):
				pass
			# travel to aux point
			# self.aux_point
			self.fsm_transit(8)

			# self.aux_point
			self.fsm_transit(8)

		if self.fsm_state == 8:
			# listen to pinger
			print "listening to pingers"
			while(self.listen_gate_pinger() ==  False):
				pass
			self.fsm_transit(9)
			
		if self.fsm_state == 9:
			

			if self.target_gate == 0:
				gate = self.gate_0
			elif self.target_gate == 1:
				gate = self.gate_1
			elif self.target_gate == 2:
				gate = self.gate_2
			# travel to target gate and start point
			# gate, self.start_point

			print "pass through the gate"
			self.add_waypoint(gate[0], gate[1], yaw=self.direction_to_gate) # not sure whether yaw is correct
			while self.status is 0:
				pass

			print "back to start point"
			direction_to_aux = math.atan2(self.start_point[1] - gate[1], self.start_point[0] - gate[0])
			self.add_waypoint(self.start_point[0], self.start_point[1], yaw=direction_to_aux) # not sure whether yaw is correct
			while self.status is 0:
				pass

			self.fsm_transit(10)
		time.sleep(1)


def main(args):
	rospy.init_node('task2_node', anonymous = True)
	ic = task2_node()
	while(1):
		ic.process()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
