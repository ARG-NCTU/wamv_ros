#! /usr/bin/env python3
import os
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Joy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, String, Float32MultiArray, Float32

class HsiAngleNav(object):
    def __init__(self):
        super().__init__()
        self.max_dis = 10  # meters
        self.laser_n = 4
        self.pos_n = 10
        self.frame = rospy.get_param("~frame", "map")
        self.target_angle = rospy.get_param("~target_angle", 0.26)
        #0.5

        self.count = 0
        self.wildlife_pose = None
        self.x = 0
        self.y = 0
        
        self.angle = 0
        self.alignment_action = False
        self.angle_state = Bool()
        self.angle_state.data = False

        self.vel_ratio = 0

        # pub cmd
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber("localization_gps_imu/pose", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_goal = rospy.Subscriber("wildlife_pose", PoseStamped, self.cb_wildlife_pose, queue_size=1)
        self.sub_alignment_action = rospy.Subscriber("alignment_action", Bool, self.cb_alignment_action, queue_size=1)
        # subscriber, timer

        self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)

    
    def cb_alignment_action(self, msg):
        self.alignment_action = msg.data

    def cb_wildlife_pose(self, msg):
        if msg.header.frame_id != self.frame:
            self.wildlife_pose = None
            return

        self.wildlife_pose = np.array([
            msg.pose.position.x, msg.pose.position.y])

    def cb_odom(self, msg):
        if self.wildlife_pose is None:
            return
        new_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y])
        
        diff = self.wildlife_pose - new_pos
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
        if self.alignment_action == False:
            return

        if self.alignment_action == True:
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
                if(self.count%51)<=25:
                    print("moving front")
                    cmd = Twist()
                    cmd.linear.x = 0.25
                    cmd.angular.z = 0
                else:
                    print("moving back")
                    cmd = Twist()
                    cmd.linear.x = -0.4
                    cmd.angular.z = 0
                if self.count == 50:
                    # self.goal = None
                    self.count = 0
            self.pub_cmd.publish(cmd)
            

if __name__ == "__main__":
    rospy.init_node("hsi_angle_nav")
    hsiangleNav = HsiAngleNav()
    rospy.spin()
