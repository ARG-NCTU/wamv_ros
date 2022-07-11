#!/usr/bin/env python

import sys
import json
import pymodbus
from pymodbus.client.sync import ModbusSerialClient as ModbusClient 

import rospy
from sensor_msgs.msg import Joy
from robotx_msgs.msg import roboteq_drive
import numpy as np
class MyDriver:
  def __init__(self):
    self.last_cmd = [0,0]
    self.count = 0
    self.safe_flag = True
    self.client =  ModbusClient(method='rtu',\
                                port='/dev/ttyUSB0',\
                                parity='N',\
                                stopbits=2,\
                                bytesize=8,\
                                baudrate=115200,\
                                timeout=3)
    if not self.client.connect():
      sys.exit("Find no Modbus server")

    self.slave_id = [0x01, 0x02]
    self.static_info = {}
    self.rt_info = {}
    # Retrieve static infos
    for sid in self.slave_id:
      rq = self.client.read_holding_registers(address=0x0000,\
                                              count=16,\
                                              unit=sid)
      assert(rq.function_code < 0x80)
      info = {}
      info['version'] = hex(rq.registers[1])
      info['pwm_rate'] = rq.registers[11]
      info['max_current'] = rq.registers[12]*0.01
      label = "sid_" + hex(sid)
      self.static_info[label] = info

      # Initial real-time infos
      info_init = {}
      info_init['rt_pwm'] = 0
      info_init['rt_current'] = 0
      self.rt_info[label] = info_init

    print "Dumping driver infos:"
    print(json.dumps(self.static_info, indent=2))
    
    self.joystick_ready = [False, False]
    self.sub = rospy.Subscriber("/joy", Joy, self.joystick_callback, queue_size=3)
    self.sub_cmd = rospy.Subscriber("/cmd_drive", roboteq_drive, self.cmd_callback, queue_size=1)
  def get_real_time_info(self):
    rt_info = {}
    for sid in self.slave_id:
      rq = self.client.read_holding_registers(address=0x0010,\
                                              count=2,\
                                              unit=sid)
      assert(rq.function_code < 0x80)
      label = "sid_" + hex(sid)
      self.rt_info[label]['rt_pwm'] = rq.registers[0]
      self.rt_info[label]['rt_current'] = rq.registers[1]*0.01

    return self.rt_info

  def set_pwm_duty_cycle(self, duty):
    for indx in range(2):
      if duty[indx] < 0:
        duty[indx] = duty[indx] & 0xffff
      rq = self.client.write_register(address=0x0040,\
                                      value=duty[indx],\
                                      unit=self.slave_id[indx])
  def cmd_callback(self, msg):
    msg.right = msg.right
    msg.left = msg.left
    if self.safe_flag == False:
      if msg.right > 999:
        msg.right = 999
      if msg.right < -999:
        msg.right = -999
      if msg.left > 999:
        msg.left = 999
      if msg.left < -999:
        msg.left = -999
      duty = [-int(msg.right), -int(msg.left)]
       
      self.set_pwm_duty_cycle(duty)
  def joystick_callback(self, msg):
    if msg.buttons[7] == 1:
      self.safe_flag = not self.safe_flag
      if self.safe_flag == True:
        print "joystick mode"
      else:
        print "autonomous mode"
    if self.safe_flag == True:
      a = -msg.axes[1]
      alpha = -msg.axes[3]*0.785
      left =  1*int(((2*a - alpha*2.44)/2)*600)
      right  = 1*int(((2*a + alpha*2.44)/2)*600)
      if left > 999:
        left = 999
      if left < -999:
        left = -999
      if right > 999:
        right = 999
      if right < -999:
        right = -999
      duty = [left, right]
      self.count = self.count + 1
      if self.count % 5 == 0:
      	print duty
      	self.set_pwm_duty_cycle(duty)

  def dipose(self):
    self.set_pwm_duty_cycle([0, 0])
    self.client.close()

if __name__ == '__main__':
  rospy.init_node('aqmd6030ns', anonymous=False)
  aqmd6030ns = MyDriver()
  
  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    #info = aqmd6030ns.get_real_time_info()
    #rospy.logwarn(info)
    rate.sleep()

  aqmd6030ns.dipose()
