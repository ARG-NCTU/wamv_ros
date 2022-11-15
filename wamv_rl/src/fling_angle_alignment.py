#! /usr/bin/env python3
import os
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Joy
from std_msgs.msg import Bool, String, Float32MultiArray, Float32, Byte, Int16

class AngleNav(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.frame = rospy.get_param("~frame", "odom")

        self.count = 0
        self.x = 0
        self.y = 0
        self.bb_x_min = 600
        self.bb_x_max = 1100
        
        self.shooting_command = Int16()
        self.shooting_command.data = 0
        self.shooting_command_arduino = Byte()
        self.shooting_command_arduino.data = 0
        self.angle = 0
        self.fling_finish_state = False
        self.nav_fling_action = True

        self.vel_ratio = 0

        # pub cmd
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_shoot = rospy.Publisher("shoot", Int16, queue_size=1)
        self.pub_shoot_arduino = rospy.Publisher("shooting", Byte, queue_size=1)
        self.pub_fling_finish_state = rospy.Publisher("fling_finish_state", Bool, queue_size=1)
        self.sub_bb_info = rospy.Subscriber("bb_info", Float32MultiArray, self.cb_bbinfo)
        self.sub_navi_fling_action = rospy.Subscriber("nav_to_fling_action", Bool, self.cb_navi_fling_action)
        # subscriber, timer

        self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)

    def cb_bbinfo(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
    
    def cb_navi_fling_action(self, msg):
        self.nav_fling_action = msg.data

    def inference(self, event):
        if self.nav_fling_action == True:
            return

        if self.nav_fling_action == False:
            if(self.bb_x_min <= self.x <= self.bb_x_max):
                if(self.count < 120):
                    print("angle correct")
                    self.fling_finish_state = False
                    cmd = Twist()
                    cmd.linear.x = 0
                    cmd.angular.z = 0.15
                    self.pub_cmd.publish(cmd)
                    self.count += 1
                    self.pub_fling_finish_state.publish(self.fling_finish_state)
                else:
                    print("end")
                    self.fling_finish_state = True
                    cmd = Twist()
                    cmd.linear.x = 0
                    cmd.angular.z = 0
                    self.pub_cmd.publish(cmd)
                    self.shooting_command.data = 0
                    self.pub_shoot.publish(self.shooting_command)
                    self.pub_fling_finish_state.publish(self.fling_finish_state)
                    return

                if(self.count%30)<=15:
                    if((self.count%30)==1):
                        print("Let's Go shooting")
                        self.shooting_command.data+=1
                        self.shooting_command_arduino.data=1
                        self.pub_shoot_arduino.publish(self.shooting_command_arduino)
                        rospy.sleep(0.5)
                        self.pub_shoot.publish(self.shooting_command)
                        rospy.sleep(0.5)
                        
                        # rospy.sleep(1.0)
                        self.shooting_command_arduino.data=0
                        self.pub_shoot_arduino.publish(self.shooting_command_arduino)
                # else:
                #     print("stop shooting")
                #     self.shooting_command.data = 0
                #     self.pub_shoot.publish(self.shooting_command)

            elif(self.x<self.bb_x_min):
                self.fling_finish_state = False
                cmd = Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0.3
                self.pub_cmd.publish(cmd)
                print("adjust left")
                return

            elif(self.x>self.bb_x_max):
                self.fling_finish_state = False
                cmd = Twist()
                cmd.linear.x = 0
                cmd.angular.z = -0.3
                self.pub_cmd.publish(cmd)
                print("adjust right")
                return
            
            self.pub_fling_finish_state.publish(self.fling_finish_state)

if __name__ == "__main__":
    rospy.init_node("angle_nav")
    angleNav = AngleNav()
    rospy.spin()
