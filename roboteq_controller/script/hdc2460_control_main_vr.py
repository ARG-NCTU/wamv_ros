#!/usr/bin/env python3

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

    self.sub = rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
    self.sub = rospy.Subscriber("/cmd_drive", roboteq_drive, self.CmdCallback, queue_size=1)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()
    self.safe_flag = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
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
      print self.ch1_pwm, self.ch2_pwm
      # Motor's pwm command
      command = "!M " + str(self.ch1_pwm) +\
                " " + str(self.ch2_pwm) + "\r"
      self.port.write(command)
      self.port.flush()
      rate.sleep()

  def CmdCallback(self, msg):
    if self.safe_flag == False:
      print "cb"  
      if msg.left<990 and msg.left>-990:
        self.ch2_pwm = msg.left
      else:
        self.ch2_pwm = (msg.left/abs(msg.left))*990
      if msg.right<990 and msg.right>-990:
        self.ch1_pwm = msg.right
      else:
        self.ch1_pwm = (msg.right/abs(msg.right))*990
  
  def JoyCallback(self, msg):
   
    if msg.buttons[7] == 1:
        self.safe_flag = not self.safe_flag
        if self.safe_flag == True:
          print "joystick mode"
        else:
          print "autonomous mode"
    if self.safe_flag == True:
        a = msg.axes[1]
        alpha = msg.axes[3]*0.785
        self.ch1_pwm = int(((2*a-alpha*2.44))*400)
        self.ch2_pwm = int(((2*a+alpha*2.44))*400)
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
