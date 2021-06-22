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
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped
from robotx_msgs.msg import ObjectPoseList

# Daniel's waypoint control
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from robotx_msgs.msg import Waypoint, WaypointList
from robotx_msgs.srv import *

#from pinger_angle_estimation.srv import *

from robotx_msgs.msg import SemiState


# Choice of Direction Of Arrival algorqithm
ALGORITHM_DOA = 'tdoa_spl_douFilter'

class gate_passing_node():
	def __init__(self):
		# ros related
		self.odom_sub		= rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
		self.obj_sub		= rospy.Subscriber("/obj_list/map", ObjectPoseList, self.objlist_cb)
		self.sub_waypoint_status = rospy.Subscriber("/wp_nav_state", Int32, self.wpstatus_cb)
		self.sub_new_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.call_new_goal, queue_size=1)
		self.pub_gate_passed = rospy.Publisher("/gate_passed", SemiState,  queue_size=1)
		self.sub_pinger_angle = rospy.Subscriber("/guess_angle", Float32, self.get_angle_cb, queue_size=1)

		self.is_added= False
		self.x = 0
		self.y = 0
		self.yaw = 0
		self.objlist = None
		self.fsm_state = 1
		self.gate_0 = [0, 0]
		self.gate_1 = [0, 0]
		self.gate_2 = [0, 0]
		self.start_point = [0, 0]
		self.aux_point = [0, 0]
		self.target_gate = 0
		self.target_totem = [0, 0]
		self.get_goal = False
		self.goal_point = None
		self.goal_orientation = None
		self.wpstatus = 1    # Flag for checking whether WAM-V is in station keeping condition
		self.direction_to_gate = 0

		self.found_target_totem = False
		self.equip_dict = {'red_totem': [0.0, 0.0, 0.0],
							'white_totem1': [0.0, 0.0, 0.0],
							'white_totem2': [0.0, 0.0, 0.0],
							'green_totem': [0.0, 0.0, 0.0],
							'black_totem': [0.0, 0.0, 0.0],
							'light_totem': [0.0, 0.0, 0.0],
		}
		# print self.equip_dict

		self.end_time = 0
		self.start_time = 0

		self.cnt_pub = 0

		# gate vector red_totem point to green_totem
		self.vector_gate = [0, 0]

		self.msg = SemiState()
		self.angle_list = []



		# rospy.Timer(rospy.Duration(2), self.updateEquip_cb)


	def get_angle_cb(self, msg):
		self.angle_list.append(msg.data)
		print 'get angle', msg.data


	def updateEquip_cb(self, event):
		OBJ_SCAN_RADIUS = 1.5

		# For each equipment, update their global position because of GPS drifting 
		for equip_name, equip_pos in self.equip_dict.items():
			if equip_pos != [0, 0]:
				for i in range(len(self.objlist.list)):
					obj_x = self.objlist.list[i].position.x
					obj_y = self.objlist.list[i].position.y
					
					if self.objlist.list[i].type == 'totem' and self.distance([obj_x, obj_y], equip_pos) <= OBJ_SCAN_RADIUS:
						self.equip_dict[equip_name] = [obj_x, obj_y]
						# print 'obj: ', [obj_x, obj_y]
						# print 'eq: ', equip_pos



	def wpstatus_cb(self, msg):
		self.wpstatus = msg.data

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
		angle = -50.0

		self.find_gate_pos()
		self.find_start_and_aux_point()
		
		angle = np.median(self.angle_list)
		print 'gate angle = ', angle

		try:
		# 	angle_estimate = rospy.ServiceProxy("/tdoa_spl_douFilter", AngleEstimation)
		# 	angle_srv = AngleEstimationRequest()
		# 	angle_srv.command = 0
			
		# 	# res = AngleEstimationResponse()
		# 	# rospy.wait_for_service("/tdoa_spl_douFilter")
		# 	# res.angle = angle_estimate(angle_srv)
		# 	# angle = res.angle
		# 	# print angle
			self.equip_dict['red_totem'][2]

			rad = self.yaw + angle * np.pi / 180.0
			# if int(angle) in range(int(self.equip_dict['red_totem'][2]), int(self.equip_dict['white_totem1'][2])): 
			# 	self.target_gate = 0
			# elif int(angle) in range(int(self.equip_dict['white_totem1'][2]), int(self.equip_dict['white_totem2'][2])):
			# 	self.target_gate = 1
			# elif int(angle) in range(int(self.equip_dict['white_totem2'][2]), int(self.equip_dict['green_totem'][2])):
			# 	self.target_gate = 2
			# else:
			# 	self.target_gate = 3
			if angle > self.equip_dict['green_totem'][2] and angle < self.equip_dict['white_totem2'][2]:
				self.target_gate = 2
			elif angle > self.equip_dict['white_totem2'][2] and angle < self.equip_dict['white_totem1'][2]:
				self.target_gate = 1
			elif angle > self.equip_dict['white_totem1'][2] and angle < self.equip_dict['red_totem'][2]:
				self.target_gate = 0
			else:
				self.target_gate = 3
				return False	
			print 'detected gate', self.target_gate
		except rospy.ServiceException, e:
			print "Task2 service call failed: %s"%e
				
		return True

	def distance(self, a, b):
		return np.sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]))

	def avg(self, a, b):
		return [(a[0]+b[0])/2, (a[1]+b[1])/2]

	def angleP2P(self, pos, goal):
		a = math.degrees(math.atan2(goal[1] - pos[1], goal[0] - pos[0]))
		# a  = math.degrees(math.atan2(goal[1] - pos[1], goal[0] - pos[0]))
		# if a < 0:
		   # a = a+360
		# print a, 'degress'
		# rospy.sleep(1)
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
			print "Task2: set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw / math.pi * 180.0, 'degrees'
			# rospy.sleep(1)

		except rospy.ServiceException, e:
			print "Task2 service call failed: %s"%e

		start_motion = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
		start_motion()
		rospy.sleep(5)

	def find_start_and_aux_point(self):
		# SCALE = 10

		# d = self.distance(self.gate_0, self.gate_1)
		# vec = [(self.gate_1[1] - self.gate_0[1])/d, -(self.gate_1[0] - self.gate_0[0])/d]
		# candidate1 = [self.gate_1[0]+vec[0]*SCALE, self.gate_1[1]+vec[1]*SCALE]
		# candidate2 = [self.gate_1[0]-vec[0]*SCALE, self.gate_1[1]-vec[1]*SCALE]
		# if self.distance([self.x, self.y], candidate1) > self.distance([self.x, self.y], candidate2):
		# 	self.start_point = candidate2
		# 	self.aux_point = candidate1
		# else:
		# 	self.start_point = candidate1
		# 	self.aux_point = candidate2
		self.aux_point[0] = (self.equip_dict['white_totem1'][0] + self.equip_dict['white_totem2'][0]) - self.x
		self.aux_point[1] = (self.equip_dict['white_totem1'][1] + self.equip_dict['white_totem2'][1]) - self.y

	def find_gate_pos(self):
		# gate 0: red-white1
		# gate 1: white1-white2
		# gate 2: white2-green


		self.found_target_totem = False
		# find totem via distance 
		totem_list = []
		for i in range(len(self.objlist.list)):
			if self.objlist.list[i].type == 'totem':
				d = self.distance([self.x, self.y], [self.objlist.list[i].position.x, self.objlist.list[i].position.y])
				theta = self.angleP2P([self.x, self.y], [self.objlist.list[i].position.x, self.objlist.list[i].position.y])
				totem_list.append([self.objlist.list[i], theta, d])

		if len(totem_list) < 4:
			print 'Cannot find all gate totems for task2'
			return False
		elif len(totem_list) < 5:
			print 'Cannot find all totems for task2, but fine.'
		else:
			print 'found all totems'
			self.found_target_totem = True

		

		# collect 5 totems ,regard 4 'closest' totem as gate_list and regard farthest one as target totem  
		gate_list = sorted(totem_list, key=lambda totem: totem[2])
		# for i in range(len(gate_list)):
		# 	print 'd= ', gate_list[i][2]

		if self.found_target_totem:
			self.equip_dict['black_totem'] = [gate_list[4][0].position.x, gate_list[4][0].position.y]

		gate_list = sorted(gate_list[0:4], key=lambda totem: totem[1]) 

		# for i in range(len(gate_list)):
			# print 'gate,', i, ' : x=', gate_list[i][0].position.x, ', y=', gate_list[i][0].position.y, ', d=', gate_list[i][2], ', theta=', gate_list[i][1]
		
		self.equip_dict['green_totem'] = [gate_list[0][0].position.x, gate_list[0][0].position.y, self.angleP2P([self.x, self.y], [gate_list[0][0].position.x, gate_list[0][0].position.y])]
		self.equip_dict['white_totem2'] = [gate_list[1][0].position.x, gate_list[1][0].position.y, self.angleP2P([self.x, self.y], [gate_list[1][0].position.x, gate_list[1][0].position.y])]
		self.equip_dict['white_totem1'] = [gate_list[2][0].position.x, gate_list[2][0].position.y, self.angleP2P([self.x, self.y], [gate_list[2][0].position.x, gate_list[2][0].position.y])]
		self.equip_dict['red_totem'] = [gate_list[3][0].position.x, gate_list[3][0].position.y, self.angleP2P([self.x, self.y], [gate_list[3][0].position.x, gate_list[3][0].position.y])]
		
		d = self.distance([self.equip_dict['white_totem1'][0], self.equip_dict['white_totem1'][1]], [self.equip_dict['white_totem2'][0], self.equip_dict['white_totem2'][1]])
		self.msg.gate_vector.x = (self.equip_dict['white_totem2'][0] - self.equip_dict['white_totem1'][0]) / d
		self.msg.gate_vector.y = (self.equip_dict['white_totem2'][1] - self.equip_dict['white_totem1'][1]) / d
		self.msg.gate_vector.z = 0
		print 'boat: x=', self.x, ', y=', self.y, ', yaw=', self.yaw

		for name, p in self.equip_dict.items():
			print name, ' pose=', p, ' theta= ',self.angleP2P([self.x, self.y], p) 

		# calculate gate positions
		self.gate_0 = self.avg(self.equip_dict['red_totem'], self.equip_dict['white_totem1'])
		self.gate_1 = self.avg(self.equip_dict['white_totem1'], self.equip_dict['white_totem2'])
		self.gate_2 = self.avg(self.equip_dict['white_totem2'], self.equip_dict['green_totem'])

		print 'gate_0: ', self.gate_0
		print 'gate_1: ', self.gate_1
		print 'gate_2: ', self.gate_2
 
		# checking if the positions are correct
		if self.equip_dict['red_totem'] == [0, 0] or self.equip_dict['white_totem1'] == [0, 0] or \
			self.equip_dict['white_totem2'] == [0, 0] or self.equip_dict['green_totem'] == [0, 0]:
			print "some totems are not detected"
			rospy.sleep(2)
			return False

		gate_width_0 = np.abs(self.distance(self.equip_dict['red_totem'], self.equip_dict['white_totem1']))
		gate_width_1 = np.abs(self.distance(self.equip_dict['white_totem1'], self.equip_dict['white_totem2']))
		gate_width_2 = np.abs(self.distance(self.equip_dict['white_totem2'], self.equip_dict['green_totem']))
		gate_width_avg = (gate_width_0 + gate_width_1 + gate_width_2)/3

		ERROR_GATE_WIDTH = 6
		if np.abs(gate_width_avg - gate_width_0) > ERROR_GATE_WIDTH or np.abs(gate_width_avg - gate_width_1) > ERROR_GATE_WIDTH or np.abs(gate_width_avg - gate_width_2) > ERROR_GATE_WIDTH:
			print "gate detection error"
		# 	rospy.sleep(2)
		# 	return False
		# while True:
		# 	rospy.sleep(5)

		return True

	def call_new_goal(self, p):
		self.goal_point = [p.pose.position.x, p.pose.position.y]
		q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
		euler = tf.transformations.euler_from_quaternion(q)
		self.goal_orientation = euler[2]
		self.get_goal = True

	def process(self):
		if self.objlist is None or self.objlist.size is 0:
			rospy.sleep(1)
			print 'no obj detect'
			return

		if self.fsm_state == 0:
			# get objlist to find gates
			print "finding gates"
			while(self.find_gate_pos() == False):
				pass
			self.fsm_transit(1)

		if self.fsm_state == 1:
			print "finding start and aux points"
			if self.get_goal:
				if not self.is_added:
					self.add_waypoint(self.goal_point[0], self.goal_point[1], yaw=self.goal_orientation) # not sure whethe+
					self.is_added = True
				if self.wpstatus is 1 and np.abs(self.yaw - self.goal_orientation) < 30*math.pi/180.0:
					self.end_time = rospy.get_time()
					if self.end_time - self.start_time > 3:
						print 'orientation ok!!'
						self.get_goal = False
						self.fsm_transit(2)

				else:
					self.start_time = rospy.get_time()

		if self.fsm_state == 2:
			# listen to pinger
			print "listening to pingers"
			while(self.listen_gate_pinger() ==  False):
				pass
			self.cnt_pub = 0
			self.fsm_transit(3)

		if self.fsm_state == 3:
			gate = [0.0, 0.0]
			if self.target_gate == 0:
				gate = self.gate_0
			elif self.target_gate == 1:
				gate = self.gate_1
			elif self.target_gate == 2:
				gate = self.gate_2
			else:
				fsm_transit(2)

			# travel to target gate and aux point
			# gate, self.aux_point
			

			print "pass through the gate (pre-point)"
			self.direction_to_gate = self.yaw
			self.add_waypoint((gate[0]+self.x)/2, (gate[1]+self.y)/2, yaw=self.angleP2P([self.x, self.y], [(gate[0]+self.x)/2, (gate[1]+self.y)/2]))
			while self.wpstatus is 0:
				pass
			print "pass through the gate"
			self.add_waypoint(gate[0], gate[1], yaw=self.direction_to_gate) # not sure whether yaw is correct
			while self.wpstatus is 0:
				pass

			print "travel to aux point"
			self.add_waypoint(self.aux_point[0], self.aux_point[1], yaw=self.direction_to_gate) # not sure whether yaw is correct
			while self.wpstatus is 0:
				pass

			
			while True:
				if self.cnt_pub < 5:
					self.cnt_pub = self.cnt_pub + 1
					self.msg.gate_passed = True
					self.pub_gate_passed.publish(self.msg)
				rospy.sleep(1)

		rospy.sleep(0.5)


def main(args):
	rospy.init_node('gate_passing_node', anonymous = True)
	ic = gate_passing_node()
	while(1):
		ic.process()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)