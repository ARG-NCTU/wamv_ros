#!/usr/bin/env python

import numpy as np
import math
import time
# Ros Library
import rospy
from robotx_msgs.msg import ObjectPoseList, ObjectPose
from robotx_msgs.msg import Waypoint
from robotx_msgs.srv import waypoint, waypointRequest, waypointResponse
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

class task6_node():
    def __init__(self):
        self.dock = None
        self.wamv_pose = None
        self.fsm_state = -1
        self.obj_list = None
        self.status = 1
        self.start = False
        self.depart = False
        self.now_waypt = 0

        rospy.Timer(rospy.Duration(0.1), self.process)

        # RosService
        self.srv_start= rospy.Service('~start', Trigger, self.cb_srv_start)

        # Publisher
        self.pub_arrive = rospy.Publisher("~arrive", Bool, queue_size=1)

        # Subscriber
        self.sub_obj_list	= rospy.Subscriber("/obj_list/map", ObjectPoseList, self.cb_objlist)
        self.sub_waypoint_status = rospy.Subscriber("/wp_nav_state", Int32, self.cb_status)
        self.sub_move = rospy.Subscriber("~depart", Bool, self.cb_depart)

    def cb_objlist(self, msg):
        self.obj_list = msg
        self.wamv_pose = msg.robot

    def cb_status(self, msg):
        self.status = msg.data

    def fsm_transit(self, state_to_transit):
        self.fsm_state = state_to_transit

    def cb_srv_start(self, request):
        print ("Start")
        self.start = True
        return TriggerResponse() 

    def cb_depart(self, msg):
        self.depart = msg.data

    def add_waypoint(self, x, y, yaw):

        try:
            send_waypoint = rospy.ServiceProxy("/new_goal", waypoint)
            wp_srv = waypointRequest()
            wp_srv.waypointx = x
            wp_srv.waypointy = y
            wp_srv.yaw = yaw
            
            res = waypointResponse()
            rospy.wait_for_service("/new_goal")
            res.waypoint_len = send_waypoint(wp_srv)
            print "Task5: set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw/math.pi*180
            time.sleep(3)

        except rospy.ServiceException, e:
            print "Task5 service call failed: %s"%e

    def process(self, event):
        if self.obj_list is None:
            return
        
        dock_center_x = 0
        dock_center_y = 0
        counter = 0
        for i in range(0, len(self.obj_list.list)):
            obj = self.obj_list.list[i]
            if obj.type == "dock":
                dock_center_x += obj.position.x
                dock_center_y += obj.position.y
                counter += 1
        if counter != 0:
            dock_center_x = dock_center_x / counter
            dock_center_y = dock_center_y / counter
            self.dock = [dock_center_x, dock_center_y]

        if self.start == True and self.fsm_state == -1 and self.dock is not None:
            self.fsm_transit(1)
        print self.fsm_state

        if self.fsm_state == 1:
            # setting 4 points of dock		
            radius = 25
            pose0 = [self.dock[0] + radius, self.dock[1], -np.pi]
            pose1 = [self.dock[0], self.dock[1] + radius, -np.pi/2]
            pose2 = [self.dock[0] - radius, self.dock[1], 0]
            pose3 = [self.dock[0], self.dock[1] - radius, np.pi/2]			
            self.pose_list = [pose0, pose1, pose2, pose3]
            self.fsm_transit(2)
            return 

        if self.fsm_state == 2:
            # Traverse target pose
            target_pose = self.pose_list[self.now_waypt]
            if self.now_waypt == 5:
                self.fsm_transit(4)
                return 

            if self.status is 1:
                self.now_waypt += 1 
                self.add_waypoint(target_pose[0], target_pose[1], target_pose[2])
                self.fsm_transit(3)
                return 

        if self.fsm_state == 3:
            arrive = Bool()
            if self.status is 1:
                arrive.data = True
            else:
                arrive.data = False
            self.pub_arrive.publish(arrive)

            if self.depart is True:
            	self.depart = False
                self.fsm_transit(1)
                return 

        if self.fsm_state == 4:
            print ("Done")

    def onShutdown(self):
        rospy.loginfo("[Task6] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('task6_node', anonymous = True)
    task6 = task6_node()
    rospy.on_shutdown(task6.onShutdown)
    rospy.spin()
