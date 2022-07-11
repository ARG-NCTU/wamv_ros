#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Bool



class VisualFeedback(object):
  def __init__(self):
    self.node_name = "visual_feedback"

    self.pins = [4, 17, 27, 22, 21]    
    self.gpio_no = self.pins #gpio_no

    # Subscribers
    self.sub_red = rospy.Subscriber("~visual_fb_red", Bool, self.cbRed, queue_size=1)
    self.sub_yellow = rospy.Subscriber("~visual_fb_yellow", Bool, self.cbYellow, queue_size=1)
    self.sub_green = rospy.Subscriber("~visual_fb_green", Bool, self.cbGreen, queue_size=1)
    self.sub_bz = rospy.Subscriber("~visual_fb_bz", Bool, self.cbBz, queue_size=1)

    # Initial gpio and attach SIGINT event. 
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.gpio_no, GPIO.OUT)

    for pin in self.pins:
      self.setLed(pin, 1)

    rospy.sleep(1)

    self.reset()

    GPIO.output(21, 0)

  def reset(self):
    for pin in self.pins:
      GPIO.output(pin, 1)

  def setLed(self, pin, on): 
    data = 0 if on==True else 1
    GPIO.output(pin, data)

  def cbRed(self, msg):
    self.setLed(self.pins[0], msg.data)
 
  def cbYellow(self, msg):
    self.setLed(self.pins[1], msg.data)

  def cbGreen(self, msg):
    self.setLed(self.pins[2], msg.data)

  def cbBz(self, msg):
    self.setLed(self.pins[3], msg.data)

  def onShutdown(self):
    self.reset()
    GPIO.cleanup()
    rospy.loginfo("[Visual Feedback] Shutdown.")


if __name__ == '__main__':
  rospy.init_node('visual_feedback', anonymous=False)
  visual_feedback_node = VisualFeedback()
  rospy.on_shutdown(visual_feedback_node.onShutdown)
  rospy.spin()
