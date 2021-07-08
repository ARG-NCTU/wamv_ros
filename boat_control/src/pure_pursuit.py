#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from nav_msgs.msg import Path, Odometry
import numpy as np
import math
import tf
import copy

class PurePursuit(object):
    def __init__(self):
        self.pub_path = rospy.Publisher("demo_path", Path, queue_size=1)
        self.look_ahead_distance = 0.1
        self.look_ahead_index = 30
        self.vis_path = Path()
        self.path = None

    def set_look_ahead_distance(self, dist):
        self.look_ahead_distance = dist
    
    def set_look_ahead_idx(self, index):
        self.look_ahead_index = index

    def set_path(self, path):
        self.vis_path = copy.deepcopy(path)
        self.path = np.zeros([len(path.poses), 2])
        for i in range(len(path.poses)):
            self.path[i] = [path.poses[i].pose.position.x,
                            path.poses[i].pose.position.y]

    def get_goal(self, current_pose):  # current_pose : PoseStamped
        if self.path is None:
            rospy.logwarn("purepursuit : no path")
            return
        
        idx_to_check = min(self.look_ahead_index, self.path.shape[0])
        pose = np.repeat(
            [[current_pose.pose.position.x, current_pose.pose.position.y]], idx_to_check, axis=0)
        dist = np.linalg.norm(pose-self.path[:idx_to_check], axis=1)
        idx = np.argmin(dist)
        
        pose = pose[idx:]
        unfinished = self.path[idx:][:pose.shape[0]]
        dist = np.linalg.norm(pose-unfinished, axis=1)

        unfinished = unfinished[np.where(dist > self.look_ahead_distance)]
        self.path = np.vstack((unfinished, self.path[idx:][pose.shape[0]:]))
        
        self.vis_path.poses = self.vis_path.poses[-self.path.shape[0]:]
        self.pub_path.publish(self.vis_path)

        if len(unfinished) == 0:
            rospy.logwarn("purepursuit : no unfinished path")
            return

        if np.linalg.norm(pose-unfinished[-1]) < 0.3:
            rospy.loginfo("finish")
            return

        goal = PoseStamped()
        goal.pose.position.x = unfinished[0][0]
        goal.pose.position.y = unfinished[0][1]
        return goal
