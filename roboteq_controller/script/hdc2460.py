#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
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

    self.sub = rospy.Subscriber("/joy", Joy, self.JoyCallback)
    self.pub = rospy.Publisher("/hdc2460_info", hdc2460_msgs, queue_size=1)
    self.srv = rospy.Service('/hdc2460_estop', estop_srv, self.SetEstop)
    self.msg = hdc2460_msgs()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
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

      # Motor's pwm command
      command = "!M " + str(self.ch1_pwm) +\
                " " + str(self.ch2_pwm) + "\r"
      self.port.write(command)
      self.port.flush()
      rate.sleep()

  def JoyCallback(self, msg):
    # Normalize the analog range
    self.ch1_pwm = int((msg.axes[5] - 1.0) / -2.0 * 1000)
    self.ch2_pwm = int((msg.axes[2] - 1.0) / -2.0 * 1000)

    self.ch1_pwm *= -1 if msg.buttons[5] == 1 else 1
    self.ch2_pwm *= -1 if msg.buttons[4] == 1 else 1

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
