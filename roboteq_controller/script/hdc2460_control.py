#!/usr/bin/env python

import rospy
from robotx_msgs.msg import roboteq_drive
from std_msgs.msg import *
from roboteq_controller.msg import hdc2460_msgs
from roboteq_controller.srv import estop_srv, estop_srvResponse
import re
import serial

class MyDriver:
  def __init__(self):
    self.port = serial.Serial(\
                  "/dev/ttyUSB0", 115200, timeout=1,\
                  parity=serial.PARITY_NONE,\
                  stopbits=serial.STOPBITS_ONE,\
                  bytesize=serial.EIGHTBITS)
    # Close port first for unexpected disconnection
    self.port.close()
    self.port.open()

    self.estop = False
    self.ch1_pwm = 0
    self.ch2_pwm = 0

    self.sub = rospy.Subscriber("/cmd_drive", roboteq_drive, self.CmdCallback, queue_size=1)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()
    self.safe_flag = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      
      self.msg = hdc2460_msgs()
      
      self.port.write("?A\r")
      self.port.flush()
      bytes_to_read = self.port.inWaiting()
      data = self.port.read(bytes_to_read)
      #print data
      
      match_obj = re.match( r'.*A=([0-9]+):([0-9]+).*', data)
      if match_obj != None:
        ch1 = Int8(int(match_obj.group(1)))
        ch2 = Int8(int(match_obj.group(2)))
        self.msg.motor_amp = [ch1, ch2]
      '''
      self.port.write("?V\r")
      self.port.flush() 
      bytes_to_read = self.port.inWaiting()
      data = self.port.read(bytes_to_read)
      #print data 
      
      match_obj = re.match( r'.*v=([0-9]+):([0-9]+):([0-9]+).*', data)
      if match_obj != None:
        ch1 = Int8(int(match_obj.group(1)))
        ch2 = Int8(int(match_obj.group(2)))
        ch3 = Int8(int(match_obj.group(3)))
        self.msg.battery_voltage = [ch1, ch2]
      '''
      self.pub.publish(self.msg)
       
      
      # Motor's pwm command
      command = "!M " + str(self.ch1_pwm) +\
                " " + str(self.ch2_pwm) + "\r"
      self.port.write(command)
      self.port.flush()
      
      rate.sleep()

  def CmdCallback(self, msg):
    if msg.left<1000 and msg.left>-1000:
      self.ch1_pwm = msg.left
    else:
      self.ch1_pwm = (msg.left/abs(msg.left))*1000
    if msg.left<1000 and msg.left>-1000:
      self.ch2_pwm = msg.right
    else:
      self.ch2_pwm = (msg.right/abs(msg.right))*1000
    
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
