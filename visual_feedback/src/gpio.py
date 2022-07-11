#!/usr/bin/env python
# From   : NCTU & Motorcon inc. co-host program.
# Purpose: Generate pulse sequence by the corresponding input.
# Author : Hao-Wei Hsu / IanHsu@motorcontech.com

# APIs:
import sys
import time
import math
import numpy
import signal
import threading
import RPi.GPIO as GPIO
"""
Constant values:
 kRadius : Wheel's radius (cm). 
 kEncRes : Encoder resolution.
 kSmpTime: Sampling Time (ms).
 kMaxVel : Max cartesian velocity (km/h).
 kMaxPPMS: Max Pulse Per MilliSecond.
"""
kRadius  = 8.5
kEncRes  = 1024
kSmpTime = 10
kMaxVel  = 40
kMaxPPMS = kMaxVel*kSmpTime/36.0 / (2*math.pi*kRadius) * kEncRes

class Pulse:
  def __init__(self, gpio_no):
    # @param: [pulse_pin, dir_pin, ...]

    self.gpio_no = gpio_no
    self.pulse = numpy.zeros(2, numpy.int)
    self.cond = [threading.Condition() for _ in range(2)]
    self.task_end = False
    self.threads = []
    for i in range(2):
      self.threads.append(threading.Thread(target=self.generator, args=(i,)))
      self.threads[i].start()
  
    # Initial gpio and attach SIGINT event. 
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_no, GPIO.OUT)
    signal.signal(signal.SIGINT, self.dtor)
    print "Pulse:Ctrl+C to terminate."

    GPIO.output(20, 0)

  def dtor(self, signal, frame):
    print "Pulse:Close background threads."
    for obj in self.cond:
      obj.acquire()

    self.task_end = True
    
    for obj in self.cond:
      obj.notify()
      obj.release()   

    for th in self.threads:
      th.join()
    
    print "Pulse:Clean up GPIO."
    GPIO.cleanup()
    sys.exit(0)

  def generator(self, *args):
    t_id = args[0]
    gpio = self.gpio_no[2*t_id: 2*t_id+2]
    cond = self.cond[t_id]

    while 1:
      cond.acquire()

      # Refresh data.
      pulse = self.pulse[t_id]

      if self.task_end: break
      
      if pulse != 0:
        neg_dir = 0 if pulse > 0 else 1

        GPIO.output(gpio[1], neg_dir)
        pulse = abs(pulse)

        if pulse > kMaxPPMS:
          pulse = int(kMaxPPMS)

        # 50% duty cycle time (us).
        duty_time = 500.0*kSmpTime / (2*pulse)

        for i in range(pulse):
          GPIO.output(gpio[0], 0)
          time.sleep(duty_time / 1e6)
          GPIO.output(gpio[0], 1)
          time.sleep(duty_time / 1e6)
      cond.wait()

    cond.release()

  def set_speed(self, pulse):
    #@param: [pulse_l, pulse_r]
    for obj in self.cond:
      obj.acquire()

    self.pulse = pulse
   
    for obj in self.cond:
      obj.notify()
      obj.release()       
 
import rospy
from geometry_msgs.msg import Twist
"""
Script example:
Master runs teleop_twist_keyboard node,
client acts the corresponding movement.
"""

ps = Pulse([5, 6, 13, 19, 20, 21])

def callback(msg):
  rospy.loginfo("Linear: [%f] Angular: [%f]" %(msg.linear.x, msg.angular.z))
  if msg.linear.x == 0:
    ps.set_speed([int(-0.5*msg.angular.z*kMaxPPMS), int(-0.5*msg.angular.z*kMaxPPMS)]) 
  else:
    ps.set_speed([int(-msg.linear.x*kMaxPPMS), int(msg.linear.x*kMaxPPMS)])

def listener():
  rospy.init_node('wheel_motor', disable_signals=True)
  rospy.Subscriber("/cmd_vel", Twist, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()

