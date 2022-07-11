#!/usr/bin/env python

import rospy
from robotx_msgs.msg import roboteq_drive
from sensor_msgs.msg import Joy
from std_msgs.msg import *
from roboteq_controller.msg import hdc2460_msgs
from roboteq_controller.srv import estop_srv, estop_srvResponse
import re
import serial

class MyDriver:
  def __init__(self):
    self.port = serial.Serial(\
                  "/dev/ttyACM0", 115200, timeout=1,\
                  parity=serial.PARITY_NONE,\
                  stopbits=serial.STOPBITS_ONE,\
                  bytesize=serial.EIGHTBITS)
    # Close port first for unexpected disconnection
    self.port.close()
    self.port.open()

    self.estop = False
    self.ch1_pwm = 0
    self.ch2_pwm = 0
    self.mode = 0
    self.t_past = rospy.get_time()
    self.sub = rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
    self.sub = rospy.Subscriber("/cmd_drive", roboteq_drive, self.CmdCallback, queue_size=1)
    #self.sub = rospy.Subscriber("/safe_check", Int32, self.safe_cb, queue_size=1)
    self.sub = rospy.Subscriber("/estop_dell", Int32, self.estop_cb, queue_size = 1)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.pub_mode = rospy.Publisher("/mode", Int32, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()
    self.emergency_stop = False
    self.safe_flag = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      #t_now = rospy.get_time()
      '''
      # Get driver's temperature
      self.port.write("?T\r")
      self.port.flush()
      bytes_to_read = self.port.inWaiting()
      data = self.port.read(bytes_to_read)
      
      # Retrieve temp. from regex
      match_obj = re.match( r'.*T=([0-9]+):([0-9]+):([0-9]+).*', data)
      if match_obj != None:
        mcu = Int8(int(match_obj.group(1)))
        ch1 = Int8(int(match_obj.group(2)))
        ch2 = Int8(int(match_obj.group(3)))
        self.msg = hdc2460_msgs([mcu, ch1, ch2])
      self.pub.publish(self.msg)
      '''
      if self.emergency_stop:
        rospy.loginfo("EMERGENCY STOP!!!")
        #if (t_now - self.t_past) > 2:
        # Motor's pwm command
        command = "!M " + str(0) +\
                  " " + str(0) + "\r"
        self.port.write(command)
        self.port.flush()
        rate.sleep()
      else:
        #if True:
        print self.ch1_pwm, self.ch2_pwm
        # Motor's pwm command
        command = "!M " + str(self.ch1_pwm) +\
                  " " + str(self.ch2_pwm) + "\r"
        self.port.write(command)
        self.port.flush()
        rate.sleep()

  def estop_cb(self, msg):
    if msg.data == 1:
      self.emergency_stop = True
    elif msg.data == 0:
      self.emergency_stop = False

  def safe_cb(self, msg):
    if msg.data == 1:
      self.t_past = rospy.get_time()

  def CmdCallback(self, msg):
    if self.safe_flag == False:
      msg.left *= 990
      msg.right *= 990
      print "cb"  
      if msg.left<990 and msg.left>-990:
        self.ch1_pwm = msg.left
      else:
        self.ch1_pwm = (msg.left/abs(msg.left))*990
      if msg.right<990 and msg.right>-990:
        self.ch2_pwm = msg.right
      else:
        self.ch2_pwm = (msg.right/abs(msg.right))*990
  
  def JoyCallback(self, msg):
   
    if msg.buttons[7] == 1:
        self.safe_flag = not self.safe_flag
        if self.safe_flag == True:
          print "joystick mode"
          self.mode = 0
        else:
          print "autonomous mode"
          self.mode = 1
        mode_msg = Int32()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)
    if self.safe_flag == True:
        a = msg.axes[1]
        alpha = msg.axes[3]*0.785
        self.ch1_pwm = int(((2*a-alpha*2.44))*500)
        self.ch2_pwm = int(((2*a+alpha*2.44))*500)
	if self.ch1_pwm<990 and self.ch1_pwm>-990:
          self.ch1_pwm = self.ch1_pwm
        else:
          self.ch1_pwm = (self.ch1_pwm/abs(self.ch1_pwm))*990
        if self.ch2_pwm<990 and self.ch2_pwm>-990:
          self.ch2_pwm = self.ch2_pwm
        else:
          self.ch2_pwm = (self.ch2_pwm/abs(self.ch2_pwm))*990

  def SetEstop(self, req):
    command = "!EX\r"if req.enable_estop.data else "!MG\r"
    self.port.write(command)
    self.port.flush()
    res = estop_srvResponse()
    return res

  def Dipose(self):
    self.port.close()
    
if __name__ == '__main__':
  rospy.init_node('roboteq_hdc2460', anonymous=False)
  hdc2460 = MyDriver()
  hdc2460.Dipose()
