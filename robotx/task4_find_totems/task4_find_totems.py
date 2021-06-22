#!/usr/bin/env python

import rospy
from robotx_msgs.msg import ObjectPoseList, ObjectPose
from totem_circle.srv import SetTotemCircle, SetTotemCircleRequest, SetTotemCircleResponse
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from robotx_msgs.msg import Waypoint, WaypointList
from robotx_msgs.srv import waypoint, waypointRequest, waypointResponse
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray
import tf
import tf_conversions
import math
import imp
import rospkg
import sys
import os
import time
import numpy as np
import cv2
from collections import Counter
import rospkg
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('zed_perception') + "/src/"
sys.path.append(os.path.dirname(os.path.expanduser(pkg_path)))
from color_detect_final import ColorDetectHSV

class Totem(object):
    def __init__(self, global_, local_, time, color):
        self.global_x = global_.x
        self.global_y = global_.y
        self.global_z = global_.z
        self.local_x = local_.x
        self.local_y = local_.y
        self.local_z = local_.z
        self.color = color
        self.color_record = []
        self.update_time = time
        self.arrive = False
    def print_global(self):
        print "x = ,", self.global_x, ", y = ", self.global_y

class task4_node():
    def __init__(self):
        # Variables
        self.node_name = rospy.get_name()
        self.fsm_state = -1
        self.obj_list = None
        self.totem_circle_command = []
        self.status = 1
        self.target_totem = None
        self.start = False
        self.waypt = []
        self.wamv_pose = None
        self.now_wapt = 0
        self.angle_counter = 0
        self.target_command = -1

        self.totem_list = []

        self.img = None
        self.cv_bridge = CvBridge()
        self.color_detect = ColorDetectHSV()
        self.tf_listener = tf.TransformListener()
        self.tf_transformer = tf.TransformerROS()

        # ros param
        self.sim = rospy.get_param('~sim', False)

        rospy.Timer(rospy.Duration(0.2), self.process)

        # Ros service
        self.srv_totem_cirle = rospy.Service('~set_totem_cirle', SetTotemCircle, self.cb_srv_totem_circle)
        self.srv_totem_cirle = rospy.Service('~clear', Trigger, self.cb_srv_clear)
        self.srv_totem_cirle = rospy.Service('~start', Trigger, self.cb_srv_start)

        # Publisher
        self.pub_marker = rospy.Publisher("~totem_marker", MarkerArray, queue_size=1)
        self.pub_image_mask = rospy.Publisher("~image_mask", Image, queue_size=1)
        self.pub_image_mask_compressed = rospy.Publisher("~image_mask/compressed", CompressedImage, queue_size=1)

        # Subscriber
        self.sub_obj = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.cb_objlist)
        self.sub_img = self.sub_image = rospy.Subscriber("~image_compressed", CompressedImage, self.cb_image)
        self.sub_waypoint_statud = rospy.Subscriber("/wp_nav_state", Int32, self.cb_status)
        self.sub_keyboard = rospy.Subscriber("/keyboard", String, self.keyboard, queue_size=1)

    def keyboard(self, msg):
        color = ["Red1", "Red2", "Blue", "Green", "Yellow", "Black", "White"]
        cmd = msg.data
        if(cmd <= '6' and cmd >= '0'):
            self.now_color = int(msg.data)
            print("Now setting Color = %s" %color[self.now_color])
            self.color_detect.setHSV(self.now_color, -1, -1)
        
        # H_max: q increase || a decrease   S_max: w increase || s decrease  V_max: e increase || d decrease (012)
        # H_min: t increase || g decrease   S_min: y increase || h decrease  V_min: u increase || j decrease (345)
        if(cmd == 'q'):
            self.color_detect.setHSV(self.now_color, 0, 5)
        elif(cmd == 'a'):
            self.color_detect.setHSV(self.now_color, 0, -5)
        elif(cmd == 'w'):
            self.color_detect.setHSV(self.now_color, 1, 5)
        elif(cmd == 's'):
            self.color_detect.setHSV(self.now_color, 1, -5)
        elif(cmd == 'e'):
            self.color_detect.setHSV(self.now_color, 2, 5)
        elif(cmd == 'd'):
            self.color_detect.setHSV(self.now_color, 2, -5)   

        elif(cmd == 't'):
            self.color_detect.setHSV(self.now_color, 3, 5)
        elif(cmd == 'g'):
            self.color_detect.setHSV(self.now_color, 3, -5)
        elif(cmd == 'y'):
            self.color_detect.setHSV(self.now_color, 4, 5)
        elif(cmd == 'h'):
            self.color_detect.setHSV(self.now_color, 4, -5)
        elif(cmd == 'u'):
            self.color_detect.setHSV(self.now_color, 5, 5)   
        elif(cmd == 'j'):
            self.color_detect.setHSV(self.now_color, 5, -5)  

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
            time.sleep(3)
            print "Task4: set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw/math.pi*180
            print "The status is ", self.status, "(Should be 0)"
            
        except rospy.ServiceException, e:
            print "Task4 service call failed: %s"%e

    def cb_status(self, msg):
        self.status = msg.data

    def totem_matching(self, obj, totem_list):
        # ================================================
        #   Find whether there has same totem in the list
        #   Return index or -1(None)
        # ================================================

        if totem_list is None:
            return -1
        #print obj.position.x, obj.position.y
        for i in range(0, len(totem_list)):
            totem = totem_list[i]
            _x = totem.global_x - obj.position.x
            _y = totem.global_y - obj.position.y 
            dis = math.sqrt( _x*_x + _y*_y )
            #print dis
            if dis <= 3.0:
                return i
        #print "============================"

        return -1

    def cb_objlist(self, msg):
        self.obj_list = msg
        self.wamv_pose = self.obj_list.robot

        # Update totem_list
        for obj in msg.list:
            if obj.type == "totem":
                index = self.totem_matching(obj, self.totem_list)
                if index == -1:
                    self.totem_list.append( Totem(obj.position, obj.position_local, rospy.get_time(), "NonDetect") )
                else:
                    self.totem_list[index].global_x = obj.position.x
                    self.totem_list[index].global_y = obj.position.y
                    self.totem_list[index].global_z = obj.position.z
                    self.totem_list[index].local_x = obj.position_local.x
                    self.totem_list[index].local_y = obj.position_local.y
                    self.totem_list[index].local_z = obj.position_local.z
                    self.totem_list[index].update_time = rospy.get_time()

        self.draw_totem()

    def distance(self, wamv_pose, totem):
        x_ = wamv_pose.position.x - totem.global_x
        y_ = wamv_pose.position.y - totem.global_y    
        dis = math.sqrt(x_*x_ + y_*y_)
        return dis 

    def cb_image(self, msg_img):
        self.img = self.cv_bridge.compressed_imgmsg_to_cv2(msg_img, "bgr8")
        if self.img is None:
            return

        # ================================================
        #   Find totem color and update the totem_list
        # ================================================
        for i in range(0, len(self.totem_list)):
            totem = self.totem_list[i]
            now = rospy.get_time()
            if now - totem.update_time >= 0.5:
                #print now-totem.update_time
                pass
            else:
                angle = math.atan2(totem.local_y, totem.local_x)
                angle = angle/math.pi*180+360
                angle = angle % 360
                #print anglef

                if self.distance(self.wamv_pose, totem) >= 15 or abs(angle-90) >= 55:
                    pass
                else:
                    color, img_mask, img_roi = self.color_detect.get_color_and_image_mask(self.img, \
                        totem.local_x, totem.local_y, totem.local_z, self.sim)
                    if color != "NonDetect":
                        limit = 50
                        if len(totem.color_record) < limit:
                            totem.color_record.append(color)
                            max_color = max(totem.color_record, key=totem.color_record.count)
                            totem.color = max_color

                            color_record_remove_none = list(totem.color_record)
                            color_record_remove_none = filter(lambda a: a != "NONE", color_record_remove_none)
                            second_max_color = "NONE"
                            
                            if len(color_record_remove_none)>0:
                                second_max_color = max(color_record_remove_none, key=color_record_remove_none.count)
                            if max_color is "NONE":
                                totem.color = second_max_color
                            if totem.color == "Black" and len(totem.color_record) >= 20:
                                totem.color = "Blue"

                            self.totem_list[i].color = totem.color
                            #print self.totem_list[i].color
                    
                        msg_img_mask = Image()
                        msg_img_mask = self.cv_bridge.cv2_to_imgmsg(img_mask, "bgr8")
                        self.pub_image_mask.publish(msg_img_mask)

    def cb_srv_totem_circle(self, request):
        # Ros service for set totem circle
        self.totem_circle_command.append([request.color, request.clockwise])
        print ("Add totem circle [color] = " + request.color+ " [clockwise] = " + str(request.clockwise))
        return SetTotemCircleResponse()

    def cb_srv_clear(self, request):
        self.totem_circle_command = []
        print ("Clear")
        return TriggerResponse()
    
    def cb_srv_start(self, request):
        print ("Start")
        self.start = True
        return TriggerResponse() 

    def fsm_transit(self, state_to_transit):
        self.fsm_state = state_to_transit

    def totem_command_match(self, color):
        for i in range(0, len(self.totem_circle_command)):
            cmd = self.totem_circle_command[i]
            #if cmd[0].lower() == "blue":
                #print cmd[0].lower(), color.lower()
            if cmd[0].lower() == color.lower():
                return i
        return -1

    def add_contoller_waypoint(self, x, y):
        rospy.wait_for_service("/add_waypoint")
        try:
            send_waypoint = rospy.ServiceProxy("/add_waypoint", waypoint)
            waypoint_len = send_waypoint(x, y, 0, 0)
            print "Task5 set contoller way_point, x = ", x, ", y = ", y
            self.start_waypoint()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def start_waypoint(self):
        rospy.wait_for_service("/start_waypoint_nav")
        try:
            start_waypoint = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
            start_ = start_waypoint()
 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        

    def clear_waypoint(self):
        rospy.wait_for_service("/clear_waypoints")
        try:
            clear_waypoint = rospy.ServiceProxy("/clear_waypoints", Trigger)
            clear_ = clear_waypoint()
            print "clear waypoints"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e   

    def process(self, event):
        #print "state = ", self.fsm_state
        # command (color, clockwise)
        if self.start is True and self.obj_list is not None and self.fsm_state is -1:
            self.fsm_transit(0)

        if self.fsm_state == 0:
            # Find match totem color
            #print "Find match totem command"
            for totem in self.totem_list:
                if totem.color is not "NONE" and  totem.color is not "NonDetect":
                    index = self.totem_command_match(totem.color)
                    # already find the target color
                    if index != -1:
                        print "Find match task totem, color = ", totem.color
                        self.target_totem = totem
                        #print totem.global_x, totem.global_y
                        self.target_command = index
                        self.fsm_transit(1)
                        return 
            
            # If no match
            min_dis = 100
            index = -1
            for i in range(0, len(self.totem_list)):
                totem = self.totem_list[i]
                if (totem.color is "NonDetect" or totem.color is "NONE") and totem.arrive is False :
                    dis = self.distance(self.wamv_pose, totem)
                    if dis < min_dis:
                        min_dis = dis
                        index = i
                        self.target_totem = totem
            if index != -1:
                self.totem_list[index].arrive = True

            if self.target_totem is not None:
                self.fsm_transit(3)	

        elif self.fsm_state == 1:
            ######################################################
            ####	Calculate target waypoint
            ######################################################
            totem = self.target_totem
            self.target_totem = None
            command = self.totem_circle_command[self.target_command]
            #print len(self.totem_circle_command) 
            del self.totem_circle_command[self.target_command]
            #print len(self.totem_circle_command) 

            print ("Start circle the totem color = ", command[0], "direction = ", command[1])

            if command[1]:
                ck = -1 
            else:
                ck = 1

            angle = math.atan2(totem.global_y- self.wamv_pose.position.y, \
                totem.global_x - self.wamv_pose.position.x)

            x_1 = totem.global_x - 10 * math.cos(angle) 
            y_1 = totem.global_y - 10 * math.sin(angle) 
            x_2 = totem.global_x + 10 * math.sin(angle) * ck
            y_2 = totem.global_y - 10 * math.cos(angle) * ck
            x_3 = totem.global_x + 10 * math.cos(angle) 
            y_3 = totem.global_y + 10 * math.sin(angle) 
            x_4 = totem.global_x - 10 * math.sin(angle) * ck
            y_4 = totem.global_y + 10 * math.cos(angle) * ck  			
            #self.waypt.append([x_1, y_1])
            self.waypt = []
            self.waypt.append([x_2, y_2])
            self.waypt.append([x_3, y_3])
            self.waypt.append([x_4, y_4])
            self.waypt.append([x_1, y_1])

            self.clear_waypoint()   
            for way_point in self.waypt:
                x = way_point[0]
                y = way_point[1]
                self.add_contoller_waypoint(x, y)   
                x_ = self.wamv_pose.position.x - x
                y_ = self.wamv_pose.position.y - y
                while math.sqrt(x_*x_ + y_*y_) >= 5 :
                    x_ = self.wamv_pose.position.x - x
                    y_ = self.wamv_pose.position.y - y 

            self.fsm_transit(2)
        
        elif self.fsm_state == 2:

            if self.status is 1:
                self.target_totem = None
                print("Finish one mission")
                if len(self.totem_circle_command) is not 0:
                    self.fsm_transit(0)
                else:
                    self.fsm_transit(5)                              
            '''
            if self.status is 1 and self.now_wapt<=4:
                wpt = self.waypt[self.now_wapt]
                self.add_waypoint(wpt[0], wpt[1], 0)
                print self.now_wapt, " th waypoint"
                self.now_wapt += 1
            
            if self.now_wapt is 5 and self.status is 1:
                self.target_totem = None
                print("Finish one mission")
                if len(self.totem_circle_command) is not 0:
                    self.now_wapt = 0
                    self.fsm_transit(0)
                else:
                    self.fsm_transit(5)
            '''

        elif self.fsm_state == 3:
            ######################################################
            ####	Go to random totem
            ######################################################
            print ("Go to the closest totem")
            totem = self.target_totem
            angle = math.atan2(totem.global_y - self.wamv_pose.position.y, \
                totem.global_x - self.wamv_pose.position.x)

            x = totem.global_x - 8 * math.cos(angle) 
            y = totem.global_y- 8 * math.sin(angle) 
        
            self.add_waypoint(x, y, angle)

            self.fsm_transit(4)
        
        elif self.fsm_state == 4:
            x = self.target_totem.local_x
            y = self.target_totem.local_y
            angle = abs(math.atan2(y, x)) / math.pi * 180 % 360

            #print self.angle_counter

            if self.status == 1:
                if 	abs(angle - 90) < 55:
                    self.angle_counter += 1
                else:
                    self.angle_counter = 0

                if self.angle_counter>30:
                    print ("Done closest totem")
                    self.target_totem = None
                    self.angle_counter = 0
                    self.fsm_transit(0)

        elif self.fsm_state == 5:
            print "Done Task4"


    def draw_totem(self):
        marker_array = MarkerArray()
        if len(self.totem_list) is 0:
            return
        for i in range(0, len(self.totem_list)):
            totem = self.totem_list[i]
            marker = Marker()
            marker.header.frame_id = "/odom"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.id = i
            marker.pose.position.x = totem.global_x
            marker.pose.position.y = totem.global_y
            marker.pose.position.z = totem.global_z
            marker.color.a = 1.0
            #print totem.color
            if totem.color.lower() == "yellow":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif totem.color.lower() == "blue":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif totem.color.lower() == "green":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif totem.color.lower() == "red":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif totem.color.lower() == "black":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0    
            elif totem.color.lower() == "white":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0                   
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0

            marker_array.markers.append(marker)
        self.pub_marker.publish(marker_array)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('task4_node', anonymous = True)
    task4 = task4_node()
    rospy.on_shutdown(task4.onShutdown)
    rospy.spin()
