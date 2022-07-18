#!/usr/bin/env python

import rospy
from robotx_msgs.msg import roboteq_drive
from sensor_msgs.msg import Joy
from std_msgs.msg import *
from std_msgs.msg import Float32MultiArray
from roboteq_controller.msg import hdc2460_msgs
from roboteq_controller.srv import estop_srv, estop_srvResponse
import re
import serial

class MyDriver:
  def __init__(self):
    self.port = serial.Serial(\
                  "/dev/ttyACM1", 115200, timeout=1,\
                  parity=serial.PARITY_NONE,\
                  stopbits=serial.STOPBITS_ONE,\
                  bytesize=serial.EIGHTBITS)
    # Close port first for unexpected disconnection
    self.port.close()
    self.port.open()

    self.estop = False
    self.ch1_pwm = 0
    self.ch2_pwm = 0
    self.count = 0

    self.motor_adjust = 0.8

    self.sub = rospy.Subscriber("/boat/vr_joy", Float32MultiArray, self.JoyCallback, queue_size=1)
    self.sub = rospy.Subscriber("/cmd_drive", roboteq_drive, self.CmdCallback, queue_size=1)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()
    self.safe_flag = True
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
      # Motor's pwm command
      command = "!M " + str(self.ch1_pwm) +\
                " " + str(self.ch2_pwm) + "\r"
      self.port.write(command)
      self.port.flush()
      rate.sleep()

  def CmdCallback(self, msg):
    if self.safe_flag == False:
      print("cb")
      self.ch1_pwm = (msg.linear.x/abs(msg.linear.x))
      self.ch1_pwm *= 990*self.motor_adjust
      self.ch2_pwm = 0
      if not (self.ch1_pwm < 990 and self.ch1_pwm > -990):
        self.ch1_pwm = (self.ch1_pwm/abs(self.ch1_pwm))*990*self.motor_adjust
  
  def JoyCallback(self, msg):

    # mode button TBD

    #if msg.data[7] == 1:
    #    self.safe_flag = not self.safe_flag
    #    if self.safe_flag == True:
    #      print "joystick mode"
    #    else:
    #      print "autonomous mode"

    if self.safe_flag == True:
        a = msg.data[0]
        self.ch1_pwm = int(a*990*self.motor_adjust)
        self.ch2_pwm = 0

        self.count = self.count +1
        if self.count % 20 == 0 :
            print("big motor : " + str(self.ch1_pwm))
            self.count = 0

        if self.ch1_pwm<990 and self.ch1_pwm>-990 :
            self.ch1_pwm = self.ch1_pwm
        else :
            self.ch1_pwm = (self.ch1_pwm/abs(self.ch1_pwm))*990*self.motor_adjust

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
