#!/usr/bin/env python

import sys
import json
import pymodbus
from pymodbus.client.sync import ModbusSerialClient as ModbusClient 

import rospy
from sensor_msgs.msg import Joy

class MyDriver:
  def __init__(self):
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
    self.sub = rospy.Subscriber("/joy", Joy, self.joystick_callback)

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

  def joystick_callback(self, msg):
    # 2 stage condition check, for safety usage
    # Stage1: Tighten
    if not self.joystick_ready[0]:
      if msg.axes[5] == -1.0 and\
         msg.axes[2] == -1.0:
        self.joystick_ready[0] = True
      return
    # Stage2: Release
    if not self.joystick_ready[1]:
      if msg.axes[5] == 1.0 and\
         msg.axes[2] == 1.0:
        self.joystick_ready[1] = True
      return

    duty = [int((msg.axes[5] - 1.0) / -2.0*1000),\
            int((msg.axes[2] - 1.0) / -2.0*1000)]
    for indx in range(2):
      duty[indx] *= -1 if msg.buttons[5-indx] == 1 else 1
    self.set_pwm_duty_cycle(duty)

  def dipose(self):
    self.set_pwm_duty_cycle([0, 0])
    self.client.close()

if __name__ == '__main__':
  rospy.init_node('aqmd6030ns', anonymous=False)
  aqmd6030ns = MyDriver()
  
  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    info = aqmd6030ns.get_real_time_info()
    rospy.logwarn(info)
    rate.sleep()

  aqmd6030ns.dipose()
