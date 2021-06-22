#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import tf
from sensor_msgs.msg import NavSatFix
import numpy as np
import cv2
import datetime
import re
import socket
import time
final_img = None
send_stc = 0


class heartbeat:
	def __init__(self):
		now = datetime.datetime.now()
		self.msg_ID = "RXHRB"
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
		self.latitude = format(0, '.5f')
		self.ns_indicator = "N"
		self.longitude = format(0, '.5f')
		self.ew_indicator = "E"
		self.team_ID = "NCTUW"
		self.system_mode = str(1)
		self.AUV_stat = str(2)
	def update_time(self):
		now = datetime.datetime.now()
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')

	def update_gps(self, latitude, longitude):
		self.latitude = format(latitude, '.5f')
		self.longitude = format(longitude, '.5f')
		
	def update_heading(self, yaw):
		print yaw
		self.ns_indicator = "N"
		self.ew_indicator = "E"
	def update_AUV(self, state):
		self.AUV_stat = str(state)
	def update_system_state(self, state):
		self.system_mode = str(state)
	def update(self, latitude, longitude, yaw):
		self.update_time()
		self.update_gps(latitude, longitude)
		self.update_heading(yaw)
	def checksum(self, sentence):

		if re.search("\n$", sentence):
			sentence = sentence[:-1]

		nmeadata,cksum = re.split('\*', sentence)

		calc_cksum = 0
		for s in nmeadata:
			calc_cksum ^= ord(s)

		""" Return the nmeadata, the checksum from
			sentence, and the calculated checksum
		"""

		return format(calc_cksum, "02X")

	def get_sentence(self):
		sentence = self.msg_ID + "," + self.date + "," + self.time \
+ "," + self.latitude + "," + self.ns_indicator + "," + self.longitude \
+ "," + self.ew_indicator + "," + self.team_ID + "," + self.system_mode \
+ "," + self.AUV_stat + "*"

		chksum = self.checksum(sentence)
		r_sentence = "$" + sentence + chksum
		# TODO add <CR><LF>
		r_sentence = r_sentence + "\r" + "\n"
		return r_sentence

class entrance_and_exit_gates:
	def __init__(self):
		now = datetime.datetime.now()
		self.msg_ID = "RXGAT"
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
		self.team_ID = "NCTUW"
		self.active_entrance_gate = str(1)
		self.active_exit_gate = str(2)
		self.light_buoy_active = "N"
		self.light_pattern = ""
		#self.light_buoy_active = "Y"
		#self.light_pattern = "RBG"

	def update_time(self):
		now = datetime.datetime.now()
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
	def update_entrance_gate(index):
		self.active_entrance_gate = str(index)
	def update_exit_gate(index):
		self.active_exit_gate = str(index)
	def update_buoy(self, data):
		if len(data) != 3:
			self.light_buoy_active = "N"
			self.light_pattern = ""
		else:
			self.light_buoy_active = "Y"
			self.light_pattern = data
	def update(self):
		self.update_time()

	def checksum(self, sentence):

		if re.search("\n$", sentence):
			sentence = sentence[:-1]

		nmeadata,cksum = re.split('\*', sentence)

		calc_cksum = 0
		for s in nmeadata:
			calc_cksum ^= ord(s)

		""" Return the nmeadata, the checksum from
			sentence, and the calculated checksum
		"""
		return format(calc_cksum, "02X")

	def get_sentence(self):
		sentence = self.msg_ID + "," + self.date + "," + self.time \
+ "," + self.team_ID + "," + self.active_entrance_gate + "," + self.active_exit_gate \
+ "," + self.light_buoy_active + "," + self.light_pattern + "*"

		chksum = self.checksum(sentence)
		r_sentence = "$" + sentence + chksum
		# TODO add <CR><LF>
		r_sentence = r_sentence + "\r" + "\n"
		return r_sentence

class scan_the_code:
	def __init__(self):
		now = datetime.datetime.now()
		self.msg_ID = "RXCOD"
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
		self.team_ID = "NCTUW"
		self.light_pattern = "RBG"

	def update_time(self):
		now = datetime.datetime.now()
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
	def update_light_pattern(self, data):
		self.light_pattern = data
	def update(self):
		self.update_time()

	def checksum(self, sentence):

		if re.search("\n$", sentence):
			sentence = sentence[:-1]

		nmeadata,cksum = re.split('\*', sentence)

		calc_cksum = 0
		for s in nmeadata:
			calc_cksum ^= ord(s)

		""" Return the nmeadata, the checksum from
			sentence, and the calculated checksum
		"""
		return format(calc_cksum, "02X")

	def get_sentence(self):
		sentence = self.msg_ID + "," + self.date + "," + self.time \
+ "," + self.team_ID + "," + self.light_pattern + "*"

		chksum = self.checksum(sentence)
		r_sentence = "$" + sentence + chksum
		# TODO add <CR><LF>
		r_sentence = r_sentence + "\r" + "\n"
		return r_sentence

class identify_symbols_and_dock:
	def __init__(self):
		now = datetime.datetime.now()
		self.msg_ID = "RXDOK"
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
		self.team_ID = "NCTUW"
		self.shape_color = "R"
		self.shape = "TRIAN"

	def update_time(self):
		now = datetime.datetime.now()
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')

	def update_shape_color(self, selfcolor, shape):
		self.shape_color = color
		self.shape = shape

	def update(self):
		self.update_time()

	def checksum(self, sentence):

		if re.search("\n$", sentence):
			sentence = sentence[:-1]

		nmeadata,cksum = re.split('\*', sentence)

		calc_cksum = 0
		for s in nmeadata:
			calc_cksum ^= ord(s)

		""" Return the nmeadata, the checksum from
			sentence, and the calculated checksum
		"""
		return format(calc_cksum, "02X")

	def get_sentence(self):
		sentence = self.msg_ID + "," + self.date + "," + self.time \
+ "," + self.team_ID + "," + self.shape_color + "," + self.shape + "*"

		chksum = self.checksum(sentence)
		r_sentence = "$" + sentence + chksum
		# TODO add <CR><LF>
		r_sentence = r_sentence + "\r" + "\n"
		return r_sentence

class detect_and_deliver:
	def __init__(self):
		now = datetime.datetime.now()
		self.msg_ID = "RXDEL"
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')
		self.team_ID = "NCTUW"
		self.shape_color = "R"
		self.shape = "TRIAN"

	def update_time(self):
		now = datetime.datetime.now()
		self.date = format(now.day, '02d') + format(now.month, '02d') + format(now.year, '4d')[-2:]
		self.time = format(now.hour, '02d') + format(now.minute, '02d') + format(now.second, '02d')

	def update_shape_color(self, color, shape):
		self.shape_color = color
		self.shape = shape

	def update(self):
		self.update_time()

	def checksum(self, sentence):

		if re.search("\n$", sentence):
			sentence = sentence[:-1]

		nmeadata,cksum = re.split('\*', sentence)

		calc_cksum = 0
		for s in nmeadata:
			calc_cksum ^= ord(s)

		""" Return the nmeadata, the checksum from
			sentence, and the calculated checksum
		"""

		return format(calc_cksum, "02X")

	def get_sentence(self):
		sentence = self.msg_ID + "," + self.date + "," + self.time \
+ "," + self.team_ID + "," + self.shape_color + "," + self.shape + "*"
		
		chksum = self.checksum(sentence)
		r_sentence = "$" + sentence + chksum
		# TODO add <CR><LF>
		r_sentence = r_sentence + "\r" + "\n"
		return r_sentence
h = heartbeat()
e = entrance_and_exit_gates()
s = scan_the_code()
i = identify_symbols_and_dock()
d = detect_and_deliver()
def display_stc(data):
	global final_img, send_stc
	send_stc = 1
	#cv2.destroyAllWindows()

	#final_img= np.zeros((300,300,3), np.uint8)
	whitetop = np.ones((100, 300, 3), np.uint8)*255
	s_whitetop = np.ones((20, 300, 3), np.uint8)*255
	v_white = np.ones((520, 20, 3), np.uint8)*255
	ans = str(data.data).split(' ', 3)
	s.update_time()
	s.update_light_pattern(ans[0][0] + ans[1][0] + ans[2][0])
	for i in range(3):
		img= np.zeros((300,300,3), np.uint8)
		color = ans[i]
		print color
		b = 0
		g = 0
		r = 0
		if color == "Red":
			r = 255
		elif color == "Green":
			g = 255
		elif color == "Blue":
			b = 255
		else:
			pass
		img[:] = [b,g,r]
		cv2.rectangle(img, (0,0), (300, 300), (0,0,0), 2)
		img = cv2.vconcat((whitetop, img))
		img = cv2.vconcat((s_whitetop, img))
		font = cv2.FONT_HERSHEY_SIMPLEX
		textsize = cv2.getTextSize(color, font, 2, 2)[0]
		print textsize
		cv2.putText(img,color,(150-int(textsize[0]/2), 50+int(textsize[1]/2)), font, 2,(0,0,0), 2)
		cv2.rectangle(img, (0,0), (300, 100), (0,0,0), 2)
		img = cv2.vconcat((whitetop, img))
		#print img
		if i == 0:
			final_img = img
			final_img = cv2.hconcat((v_white, final_img))
			final_img = cv2.hconcat((final_img, v_white))
		else:
			final_img = cv2.hconcat((final_img, img))
			final_img = cv2.hconcat((final_img, v_white))
			
	font = cv2.FONT_HERSHEY_SIMPLEX
	textsize = cv2.getTextSize("Scan the Code", font, 3, 3)[0]
	cv2.putText(final_img,"Scan the Code",(490-int(textsize[0]/2), 50+int(textsize[1]/2)), font, 3, (0,0,0), 3)


	
	
	#print final_img.shape
	#print final_img[0][0], final_img[0][300], final_img[0][600]
	
	cv2.imshow('image',final_img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cb_gps(msg):
	h.update_time()
	h.update_gps(msg.latitude, msg.longitude)
	
def cb_nav(msg):
	quaternion = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	_, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
	h.update_time()
	h.update_heading(yaw)

def cb_seadrone_state(msg):
	h.update_AUV(msg.data)

def cb_system_state(msg):
	h.update_system_state(msg.data)

def cb_docking(msg):
	global send_dock
	send_dock = 1
	c = str(msg.data).split(' ', 2)
	i.update_time()
	i.update_shape_color(c[0], c[1])

def cb_dad(msg):
	global send_stc

	c = str(msg.data).split(' ', 2)
	d.update_time()
	d.update_shape_color(c[0], c[1])

if __name__=='__main__':
	rospy.init_node("heartbeat", anonymous=True)
	rospy.Subscriber("task5/seq", String, display_stc)
	#rospy.Subscriber("/fix", NavSatFix, cb_gps) # if using real robot
	rospy.Subscriber("/gps", NavSatFix, cb_gps) # if using gazebo
	rospy.Subscriber("/odometry/filtered", Odometry, cb_nav)
	# TODO system mode sub and pub
	rospy.Subscriber("/seadrone_state", Int32, cb_seadrone_state)
	rospy.Subscriber("/visual_feedback", Int32, cb_system_state)
	# TODO seadrone publisher
	rospy.Subscriber("/task6_placard", String, cb_docking)
	rospy.Subscriber("/task7_placard", String, cb_dad)
	soc = socket.socket()         # Create a socket object
	host = "localhost" # Get local machine name
	port = 12345                # Reserve a port for your service.
	soc.connect((host, port))
	
	while True:
		
		
		'''
		h = heartbeat()
		'''
		print h.get_sentence()
		soc.send(h.get_sentence())

		if send_stc == 1:
			soc.send(s.get_sentence())
		if send_dock = 1:
			soc.send(i.get_sentence())
		if send_dock = 1:
			soc.send(i.get_sentence())
		'''
		
		e = entrance_and_exit_gates()
		print e.get_sentence()
		soc.send(e.get_sentence())
		s = scan_the_code()
		print s.get_sentence()
		soc.send(s.get_sentence())
		i = identify_symbols_and_dock()
		print i.get_sentence()
		soc.send(i.get_sentence())
		d = detect_and_deliver()
		print d.get_sentence()
		soc.send(d.get_sentence())
		'''
		time.sleep(1)
	
