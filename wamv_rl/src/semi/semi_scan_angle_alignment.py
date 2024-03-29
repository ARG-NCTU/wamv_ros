#! /usr/bin/env python3
import os
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Joy
from scipy.spatial.transform import Rotation as R
from behavior_tree_msgs.msg import Active
from std_msgs.msg import Bool, String, Float32MultiArray, Float32

class ScanAngleNav(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.frame = rospy.get_param("~frame", "map")
        self.target_angle = rospy.get_param("~target_angle", 0.3)
        #0.5

        self.count = 0
        self.goal = None
        self.x = 0
        self.y = 0
        
        self.angle = 0
        self.angle_action = False
        self.angle_state = Bool()
        self.angle_state.data = False

        self.vel_ratio = 0

        # pub cmd
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_angle_state = rospy.Publisher("/alignment_perception_scan_success", Bool, queue_size=1)
        self.sub_pose = rospy.Subscriber("localization_gps_imu/pose", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_anglenavi_action = rospy.Subscriber("/alignment_perception_scan_active", Active, self.cb_anglenavi_action, queue_size=1)
        
        # subscriber, timer

        self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)

    
    def cb_anglenavi_action(self, msg):
        self.angle_action = msg.active

    def cb_goal(self, msg):
        if msg.header.frame_id != self.frame:
            self.goal = None
            return

        self.goal = np.array([
            msg.pose.position.x, msg.pose.position.y])

    def cb_odom(self, msg):
        if self.goal is None:
            return
        new_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y])
        
        diff = self.goal - new_pos
        r = R.from_quat([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])
        yaw = r.as_euler('zyx')[0]
        self.angle = math.atan2(diff[1], diff[0]) - yaw
        if self.angle >= np.pi:
            self.angle -= 2*np.pi
        elif self.angle <= -np.pi:
            self.angle += 2*np.pi

    def inference(self, event):
        if(abs(self.angle) >= self.target_angle):
            cmd = Twist()
            cmd.linear.x = 0
            self.angle_state.data = False
            print("angle adjusting")
            if self.angle>0:
                cmd.angular.z = 0.25
            else:
                cmd.angular.z = -0.25
        else:
            print("angle finish")
            self.count += 1
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            if self.count == 25:
                # self.goal = None
                self.count = 0
                self.angle_state.data = True

        if self.angle_action == True:
            self.pub_cmd.publish(cmd)
        # self.angle_state.data = True
        self.pub_angle_state.publish(self.angle_state)
        

if __name__ == "__main__":
    rospy.init_node("scan_angle_nav")
    scanangleNav = ScanAngleNav()
    rospy.spin()
