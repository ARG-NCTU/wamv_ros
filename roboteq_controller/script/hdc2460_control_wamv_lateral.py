#!/usr/bin/env python

import rospy
from robotx_msgs.msg import roboteq_drive
from geometry_msgs.msg import Twist
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
    self.sub_cmd = rospy.Subscriber("/wamv/cmd_vel", Twist, self.CmdCallback, queue_size=1)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()
    self.safe_flag = True
    rate = rospy.Rate(10)
    print("11111")
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
      print(self.ch1_pwm, self.ch2_pwm)
      # Motor's pwm command
      command = "!M " + str(self.ch1_pwm) +\
                " " + str(self.ch2_pwm) + "\r"
      self.port.write(command)
      self.port.flush()
      rate.sleep()
  
  def CmdCallback(self, msg):
    if self.safe_flag == False:
      print ("autonomous")
      right_lateral_data = -msg.angular.z * 990
      left_lateral_data = msg.angular.z * 990
      if left_lateral_data<990 and left_lateral_data>-990:
        self.ch2_pwm = left_lateral_data
      else:
        self.ch2_pwm = (left_lateral_data/abs(left_lateral_data))*990
      if right_lateral_data<990 and right_lateral_data>-990:
        self.ch1_pwm = right_lateral_data
      else:
        self.ch1_pwm = (right_lateral_data/abs(right_lateral_data))*990
  
  def JoyCallback(self, msg):
    if msg.buttons[7] == 1 and self.safe_flag:
        self.safe_flag = False
        print ("autonomous mode")
    elif msg.buttons[6] == 1 and not self.safe_flag:
        self.safe_flag = True
        print ("joystick mode")

    if self.safe_flag == True:
        if(msg.buttons[4]==1):
          a = msg.axes[1]
          alpha = msg.axes[3]*0.785
          self.ch1_pwm = int(((-alpha*2.44))*400)
          self.ch2_pwm = int(((alpha*2.44))*400)
          if self.ch1_pwm<800 and self.ch1_pwm>-800:
            self.ch1_pwm = self.ch1_pwm
          else:
            self.ch1_pwm = (self.ch1_pwm/abs(self.ch1_pwm))*800
          if self.ch2_pwm<800 and self.ch2_pwm>-800:
            self.ch2_pwm = self.ch2_pwm
          else:
            self.ch2_pwm = (self.ch2_pwm/abs(self.ch2_pwm))*800
    
        elif msg.buttons[4] == 0:
            self.ch1_pwm = 0
            self.ch2_pwm = 0

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
