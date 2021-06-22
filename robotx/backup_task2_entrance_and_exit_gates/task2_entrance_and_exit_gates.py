#!/usr/bin/env python

import rospy
import numpy as np

class task2_node():
	def __init__(self):
		# ros related
		self.odom_sub		= rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
		self.obj_sub		= rospy.Subscriber("/obj_list/classify", ObjectPoseList, self.objlist_cb)
		

		self.x = 0
		self.y = 0
		self.yaw = 0
		self.objlist
		self.fsm_state = 0
		self.gate_0 = [0, 0]
		self.gate_1 = [0, 0]
		self.gate_2 = [0, 0]
		self.start_point = [0, 0]
		self.aux_point = [0, 0]
		self.target_gate = 0
		self.target_totem_color = "black" # or "yellow"
		self.target_totem = [0, 0]



	def odom_cb(self, msg):
		self.x
		self.y
		self.yaw

	def objlist_cb(self, msg):
		self.objlist = msg


	def fsm_transit(self, state_to_transit):
		state_to_transit = self.fsm_state

	def listen_gate_pinger(self):
		# TODO pinger code
		self.target_gate = 0
		return True

	def distance(a, b):
		return np.sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]))
	def avg(a, b):
		return [(a[0]+b[0])/2, (a[1]+b[1])/2]
	def find_start_and_aux_point(self):
		d = self.distance(self.gate_0, self.gate_1)
		vec = [(self.gate_1[1] - self.gate_0[1])/d, -(self.gate_1[0] - self.gate_0[0])/d]
		candidate1 = [self.gate_1[0]+vec[0]*10, self.gate_1[1]+vec[1]*10]
		candidate2 = [self.gate_1[0]-vec[0]*10, self.gate_1[1]-vec[1]*10]
		if self.distance([self.x, self.y], candidate1) > self.distance([self.x, self.y], candidate2):
			self.start_point = candidate2
			self.aux_point = candidate1
		else:
			self.start_point = candidate1
			self.aux_point = candidate2
	def find_gate_pos(self):
		red_totem = [0, 0]
		white_totem
		green_totem = [0, 0]
		# gate 0: red-white1
		# gate 1: white1-white2
		# gate 2: white2-green

		for i in range(self.objlist.size):
			if objlist.list[i].type == "totem" and objlist.list[i].color == "red":
				# TODO: do error recovery if more than one red totem is detected
				# maybe get the one closest to WAMV?

				red_totem = [objlist.list[i].position.x, objlist.list[i].position.y] 
			
			else if objlist.list[i].type == "totem" and objlist.list[i].color == "green":
				green_totem = [objlist.list[i].position.x, objlist.list[i].position.y]
		
		for i in range(self.objlist.size):
			if objlist.list[i].type == "totem" and objlist.list[i].color == "white":
				tmp_pos = [objlist.list[i].position.x, objlist.list[i].position.y]
				if red_totem != [0, 0] and green_totem != [0, 0]:
					if white_totem is None:
						white_totem = [[objlist.list[i].position.x, objlist.list[i].position.y]]
					else:
						if self.distance(white_totem[0], red_totem) < self.distance(tmp_pos, red_totem):
							white_totem.append(tmp_pos)
						else:
							white_totem.insert(0, tmp_pos)

		# calculate gate positions
		self.gate_0 = self.avg(red_totem, white_totem[0])
		self.gate_1 = self.avg(white_totem[0], white_totem[1])
		self.gate_2 = self.avg(white_totem[1], green_totem)

		# checking if the positions are correct
		if red_totem == [0, 0] || white_totem[0] == [0, 0] || white_totem[1] == [0, 0] || green_totem == [0, 0]:
			print "some totems are not detected"
			return False
		gate_width_0 = np.abs(self.distance(red_totem, white_totem[0]))
		gate_width_1 = np.abs(self.distance(white_totem[0], white_totem[1]))
		gate_width_2 = np.abs(self.distance(white_totem[1], green_totem))
		gate_width_avg = (gate_width_0 + gate_width_1 + gate_width_2)/3
		if np.abs(gate_width_avg - gate_width_0) > 10 || np.abs(gate_width_avg - gate_width_1) > 10 || np.abs(gate_width_avg - gate_width_2) > 10:
			print "gate detection error"
			return False


		return True

	def get_target_totem(self, totem_color):
		
		for i in range(self.objlist.size):
			if objlist.list[i].color == totem_color:
				self.target_totem  = [objlist.list[i].position.x, objlist.list[i].position.y]
				return True
		return False

	def process(self):
		if self.fsm_state == 0:
			# get objlist to find gates
			print "finding gates"
			while(self.find_gate_pos() == False):
				pass
			self.fsm_transit(1)
		if self.fsm_state == 1:
			while(self.find_start_and_aux_point() == False):
				pass
			# travel to start point
			# self.start_point
			self.fsm_transit(2)

		if self.fsm_state == 2:
			# listen to pinger
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

			self.fsm_transit(4)
		if self.fsm_state = 4:
			# get target totem 
			print "find target totem"
			while(self.get_target_totem(self.target_totem_color) == False):
				pass
			self.fsm_transit(5)

		if self.fsm_state == 5:
			# circle the target totem and come back
			# travel to ready pose
			# circle totem once
			self.fsm_transit(6)
		
		if self.fsm_state == 6:
			# get objlist to find gates
			print "finding gates"
			while(self.find_gate_pos() == False):
				pass
			self.fsm_transit(7)

		if self.fsm_state == 7:
			while(self.find_start_and_aux_point() == False):
				pass
			# travel to aux point
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

			self.fsm_transit(10)



def main(args):
	rospy.init_node('task2_node', anonymous = True)
	ic = task2_node()
	while(1):
		ic.process()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
