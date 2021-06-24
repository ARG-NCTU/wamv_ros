#!/usr/bin/env python

import rospy
from pure_pursuit import PurePursuit
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from subt_msgs.msg import ArtifactPoseArray, ArtifactPose
import tf
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32


class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())

        self.pursuit = PurePursuit()
        self.pursuit.set_look_ahead_distance(4)
        self.target_global = None

        self.pub_pid_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)

        self.frame_id = "odom"

        # demo path
        # single circle
        cencter = [28.208, -22.517]
        radius = 10
        start_angle = 120
        self.circle_path = Path()
        self.circle_path.header.frame_id = self.frame_id
        for d in range(360):
            p = PoseStamped()
            p.pose.position.x = cencter[0] + radius * \
                math.cos(math.radians(d+start_angle))
            p.pose.position.y = cencter[1] + radius * \
                math.sin(math.radians(d+start_angle))
            self.circle_path.poses.append(p)

        # double circle
        counter_radius = 6
        counter_center = [
            cencter[0]+(radius+counter_radius)*math.cos(math.radians(start_angle)),
            cencter[1]+(radius+counter_radius)*math.sin(math.radians(start_angle))
        ]
        self.duoble_circle_path = Path()
        self.duoble_circle_path.header.frame_id = self.frame_id
        poses = []
        for d in range(360):
            p = PoseStamped()
            p.pose.position.x = counter_center[0] + counter_radius * \
                math.cos(math.radians(d+start_angle+180))
            p.pose.position.y = counter_center[1] + counter_radius * \
                math.sin(math.radians(d+start_angle+180))
            poses.append(p)

        poses.reverse()
        self.duoble_circle_path.poses = self.circle_path.poses + poses
        
        # set path
        self.path_set = self.circle_path
        self.pursuit.set_path(self.path_set)
    
        # subscriber
        rospy.Subscriber('odometry/filtered',
                         Odometry, self.setpath, queue_size=1)
        print("init done")

    def setpath(self, msg):

        p = PoseStamped()
        p.pose = msg.pose.pose
        self.target_global = self.pursuit.get_goal(p)

        if self.target_global is None:
            rospy.logwarn("reset")
            self.pursuit.set_path(self.path_set)
            return

        self.pub_pid_goal.publish(self.target_global)


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav = Navigation()
    rospy.spin()
