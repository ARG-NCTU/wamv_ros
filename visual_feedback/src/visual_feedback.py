#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Bool,Int32,String

class VisualFeedback(object):
    def __init__(self):
        self.node_name = "visual_feedback"

        self.output_pins = [17, 27, 22] #red, yellow, green

        # Initial gpio and attach SIGINT event. 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.output_pins, GPIO.OUT)
        GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.record = 0
   
        #rospy.sleep(1)
        self.turnOnAll()
        rospy.sleep(2)
        self.clearAll()
        rospy.sleep(1)
        print "Start Working"
        rospy.Timer(rospy.Duration(0.5), self.detectEmergency)

        self.pub_state = rospy.Publisher("/visual_feedback", Int32, queue_size=1)

        # Subscribers
        self.sub_red = rospy.Subscriber("~visual_fb_red", Bool, self.cbRed, queue_size=1)
        self.sub_yellow = rospy.Subscriber("~visual_fb_yellow", Bool, self.cbYellow, queue_size=1)
        self.sub_green = rospy.Subscriber("~visual_fb_green", Bool, self.cbGreen, queue_size=1)
        self.sub_mode = rospy.Subscriber("/mode", Int32, self.cbMode, queue_size=1)

    def detectEmergency(self, event):
        msg = Int32()
        if (GPIO.input(13) == 0):
            msg.data = 2
            self.setLed(self.output_pins[0], True)
            self.setLed(self.output_pins[1], False)
            self.setLed(self.output_pins[2], False)
        else:
            self.setLed(self.output_pins[0], False)
            if self.record is 0: # Joystick
                self.setLed(self.output_pins[2], False)
                self.setLed(self.output_pins[1], True)
            else:
                self.setLed(self.output_pins[1], False)
                self.setLed(self.output_pins[2], True)
            msg.data = self.record
        self.pub_state.publish(msg)

    def cbMode(self, msg):
        if (msg.data == 0): # Joystick => Yellow
            self.setLed(self.output_pins[2], False)
            self.setLed(self.output_pins[1], True)
            self.record = 0
        elif (msg.data == 1): # Autonomous => Green
            self.setLed(self.output_pins[1], False)
            self.setLed(self.output_pins[2], True)
            self.record = 1

    def turnOnAll(self):
        for pin in self.output_pins:
            self.setLed(pin, 1)
            rospy.sleep(0.2)
            print ("On: ", pin)

    def clearAll(self):
        for pin in self.output_pins:
            self.setLed(pin, 0)
            rospy.sleep(0.2)
            print ("Off: ", pin)
        print("Clear all")

    def setLed(self, pin, on): 
        data = 0 if on==True else 1
        GPIO.output(pin, data)

    def cbRed(self, msg):
        self.setLed(self.output_pins[0], msg.data)
    
    def cbYellow(self, msg):
        self.setLed(self.output_pins[1], msg.data)

    def cbGreen(self, msg):
        self.setLed(self.output_pins[2], msg.data)

    def onShutdown(self):
        self.clearAll()
        GPIO.cleanup()
        rospy.loginfo("[Visual Feedback] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('visual_feedback', anonymous=False)
    visual_feedback_node = VisualFeedback()
    rospy.on_shutdown(visual_feedback_node.onShutdown)
    rospy.spin()
