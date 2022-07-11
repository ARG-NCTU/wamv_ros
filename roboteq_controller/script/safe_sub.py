#!/usr/bin/env python

import rospy
from std_msgs.msg import *

class Safe:
  def __init__(self):
    rospy.loginfo("Start safe check")
    self.t_past = rospy.get_time()
    self.sub = rospy.Subscriber("/safe_check", Int32, self.safe_cb, queue_size=1)
    self.pub_stop = rospy.Publisher("/estop_dell", Int32, queue_size=1)
    self.counter = 0
    self.estop_flag = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      t_now = rospy.get_time()
      if (t_now - self.t_past) > 20:
        msg = Int32()
        msg.data = 1
        self.pub_stop.publish(msg)
        self.estop_flag = True
      elif self.estop_flag:
        self.counter = self.counter + 1
        msg = Int32()
        msg.data = 0
        self.pub_stop.publish(msg)
      if self.counter > 15:
        self.estop_flag = False
        self.counter = 0
      '''if not self.estop_flag:
        msg = Int32()
        msg.data = 0
        self.pub_stop.publish(msg)'''
      rate.sleep()
      

  def safe_cb(self, msg):
    if msg.data == 1:
      self.t_past = rospy.get_time()
    
if __name__ == '__main__':
  rospy.init_node('safe_sub', anonymous=False)
  safe = Safe()
  rospy.spin()
