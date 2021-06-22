#!/usr/bin/env python

# ROS Lib
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
# ROS Msg
from sensor_msgs.msg import CompressedImage, Image
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, roboteq_drive
from robotx_msgs.srv import waypoint, waypointRequest, waypointResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
# Ros Service
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

import timeit
import tf
import operator
import sys
import tf_conversions
import math
import time
import cv2
import os
import threading
import numpy as np
from collections import Counter
import random
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('zed_perception') + "/src/"
sys.path.append(os.path.dirname(os.path.expanduser(pkg_path)))
from color_detect_light_buoy import ColorDetectHSV
from robotx_gazebo.msg import UsvDrive

class task5_node():
    def __init__(self):

        self.img = None
        self.node_name = rospy.get_name()
        self.light_buoy = None
        self.target_pose = None
        self.now_pose = 0
        self.pose_list = None
        self.objlist = None
        self.angle = None
        self.cv_bridge = CvBridge()
        self.color_detect = ColorDetectHSV()
        self.fsm_state = 0
        self.initial_seq()
        self.all_seq = []
        self.wamv_pose = None
        self.status = 1
        self.mutex = threading.Lock()
        self.angle_trigger = False # make sure pose keep is stable
        self.angle_counter = 0 # make sure pose keep is stable
        self.wpt_counter = 0 # Use for send mutli-waypoint
        self.angular_speed = 0.6
        self.linear_speed = 0.3

        self.sim = rospy.get_param('~sim', False)
        self.frame_rate = rospy.get_param('~frame_rate', 7)
        
        rospy.Timer(rospy.Duration(0.13), self.process)

        #Publisher
        if self.sim:
            self.pub_motion         = rospy.Publisher("/cmd_drive", UsvDrive, queue_size=1)
        else:
            self.pub_motion         = rospy.Publisher("/cmd_drive", roboteq_drive, queue_size=1)

        self.pub_image_roi = rospy.Publisher("~image_roi", Image, queue_size=1)
        self.pub_image_roi_compressed = rospy.Publisher("~image_roi/compressed", CompressedImage, queue_size=1)
        self.pub_image_mask = rospy.Publisher("~image_mask", Image, queue_size=1)
        self.pub_image_mask_compressed = rospy.Publisher("~image_mask/compressed", CompressedImage, queue_size=1)
        self.pub_seq = rospy.Publisher("~seq", String, queue_size=1)

        self.sub_obj = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.cb_objlist)
        self.sub_camera = rospy.Subscriber("~image_compressed", CompressedImage, self.cb_img)
        self.sub_keyboard = rospy.Subscriber("/keyboard", String, self.keyboard)
        self.sub_odometry = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odom)
        self.sub_waypoint_statud = rospy.Subscriber("/wp_nav_state", Int32, self.cb_status)

    def initial_seq(self):
        self.light_seq  = []
        self.tmp_seq    = []
        self.find_seq = False
        
        self.pause_counter	 = 0
        self.otherColor_counter = 0
        self.find_pause = False
        self.pause_location = -1
        
        self.t_start  = None
        self.t_end    = None
        self.start_detect = False
        self.t_start_move = None

        self.color = None
        self.pre_color = "Nothing"
        self.tmp_color = "Nothing"
        self.change_color = False

    

    def cb_status(self, msg):
        self.status = msg.data

    def cb_objlist(self, msg):
        self.objlist = msg

    def fsm_transit(self, state_to_transit):
        self.fsm_state = state_to_transit

    def cb_img(self, msg_img):
        self.img = self.cv_bridge.compressed_imgmsg_to_cv2(msg_img, "bgr8")

    def cb_odom(self, msg):
        self.wamv_pose = msg.pose.pose

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
            print "Task5: set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw/math.pi*180
            time.sleep(3)

        except rospy.ServiceException, e:
            print "Task5 service call failed: %s"%e

    def process_seq(self):
        #print "Get color = ", self.color
        
        if self.find_seq:
            return 
            
        if self.pre_color is "Nothing":
            return
        
        # If color change, initial two params
        if self.change_color and len(self.tmp_seq) != 0:
            self.change_color = False
            value = max(set(self.tmp_seq), key=self.tmp_seq.count)
            #print "pause_counter = ", self.pause_counter, ", color counter = ", self.otherColor_counter
            if value == "NONE" and self.pause_counter < self.frame_rate*2-1 :
                pass
            elif value != "NONE" and (self.otherColor_counter + self.pause_counter) < self.frame_rate:
                pass
            else:
                if value == "NONE":
                    self.pause_location = len(self.light_seq)
                    print "Pause Location = ", self.pause_location
                else:
                    if len(self.light_seq) != 0 and value == self.light_seq[len(self.light_seq)-1]:
                        pass
                    else:
                        self.light_seq.append(value)
                        print "Seq Color Add = ", value

                self.tmp_seq = []
                self.pause_counter = 0 
                self.otherColor_counter = 0

        self.tmp_seq.append(self.color)

        if self.color == "NONE":     
            self.pause_counter += 1
        else:
            self.otherColor_counter += 1

        if len(self.light_seq) is 3:
            self.rearrange_seq()

        #print len(self.light_seq)

    def guess_seq(self):
        print("Start guess sequence")
        colors = ['Red', 'Blue', 'Green']
        while len(self.light_seq) < 3:
            rnd = random.randrange(0, 3)
            if len(self.light_seq) is not 0:
                now_color = self.light_seq[ len(self.light_seq)-1 ]
            else:
                now_color = "Nothing"
            rnd_color = colors[rnd]
            if now_color is not rnd_color:
                self.light_seq.append(rnd_color)
        self.rearrange_seq()
        
    def rearrange_seq(self):
        #####################################################
        #	Rearrange the sequence 
        #####################################################
        if self.pause_location == -1:
            self.pause_location = 3
        tmp = []
        for i in range(self.pause_location, 3):
            tmp.append(self.light_seq[i])
        for i in range(0, self.pause_location):
            tmp.append(self.light_seq[i])
        self.light_seq = tmp
        if self.light_seq[0] != self.light_seq[1] and self.light_seq[1] != self.light_seq[2] : 
            print("Done, ", self.light_seq)
            self.find_seq = True
            self.all_seq.append(tuple(self.light_seq))
        else:
            self.light_seq = []	

    def update_closest(self, pose_list):
        self.pose_list = []
        min_value = 100
        min_index = -1
        for i in range(0, 4):
            pose = pose_list[i]
            dis = math.sqrt(pose[0]*pose[0]+pose[1]*pose[1])
            if (dis < min_value):
                min_value = dis
                min_index = i
        for i in range(min_index, 4):
            self.pose_list.append(pose_list[i])
        for i in range(0, min_index):
            self.pose_list.append(pose_list[i])	

    def process(self, event):
        if self.objlist is None or self.objlist.size is 0 or self.wamv_pose is None:
            return

        #print "state = ", self.fsm_state

        # get light buoy pos
        for i in range(0, self.objlist.size):
            #print self.objlist.list[i].type
            if "light_buoy" in self.objlist.list[i].type: # currently as totem
                if self.light_buoy is None:
                    self.light_buoy = self.objlist.list[i]
                    print "Target buoy = ", self.objlist.list[i].position
                else:
                    tmp_x = self.objlist.list[i].position.x - self.light_buoy.position.x
                    tmp_y = self.objlist.list[i].position.y - self.light_buoy.position.y
                    dis = math.sqrt(tmp_x*tmp_x+tmp_y*tmp_y)
                    if dis<3:
                        #print "Update buoy position"
                        self.light_buoy = self.objlist.list[i]


        if self.light_buoy is not None and self.fsm_state == 0:
            self.fsm_transit(3)

        elif self.fsm_state == 3:
            # check whether arrive the pose and stop for 3 secs
            
            x = self.light_buoy.position_local.x
            y = self.light_buoy.position_local.y
            dis = math.sqrt(x*x+y*y)
            angle = abs(math.atan2(y, x)) / math.pi * 180 % 360
            #print "Target distance = ", dis, ", Angle = ", angle-90, " x = ", x, ", y = ", y
            #print "target local position = ", x, y


            if abs(angle - 90) < 50 and dis <= 15:
                self.angle_counter += 1
            else:
                self.angle_counter = 0

            if self.angle_counter > 50:
                self.angle_trigger = True
                
            if self.t_start_move is None:
                self.t_start_move = rospy.get_time()
                print "Timer start"

            elif rospy.get_time() - self.t_start_move >= 120:
                self.t_start_move = None
                self.angle_trigger = True
                self.guess_seq()
                self.fsm_transit(4)


            # Motion
            angle = angle - 90
            if(self.sim):
                motor_msg = UsvDrive()
            else:
                motor_msg = roboteq_drive()
            motor_msg.left = 0
            motor_msg.right = 0

            radius = float(angle) / 90.0
            base = 0
        
                          # Backward
            if(dis<=7):
                motor_msg.left  = -self.linear_speed - self.angular_speed * radius - base
                motor_msg.right = -self.linear_speed + self.angular_speed * radius - base
                #print 'Backward'
            # Pose keep
            elif(dis<=13):
                motor_msg.left  = - (self.angular_speed * radius * 1.5) 
                motor_msg.right = (self.angular_speed * radius * 1.5) 
                #print 'Pose keep'
            # Go ahead
            else:
                motor_msg.left  = self.linear_speed - self.angular_speed * radius + base
                motor_msg.right = self.linear_speed + self.angular_speed * radius + base
            '''
            if motor_msg.left - motor_msg.right > 0:
                print "Turn right",
            elif motor_msg.left - motor_msg.right < 0:
                print "Trun left",
            else:
                pass
            '''
            if not self.sim:
                motor_msg.right *= 1200 
                motor_msg.left *= 1200
                
                tmp = motor_msg.left
                motor_msg.left = motor_msg.right
                motor_msg.right = tmp

                print motor_msg.left, motor_msg.right
            if motor_msg.left >= 900:
                motor_msg.left = 900
            if motor_msg.right >= 900:
                motor_msg.right = 900
            if motor_msg.left <= -900:
                motor_msg.left = -900
            if motor_msg.left <= -900:
                motor_msg.left = -900

            self.pub_motion.publish(motor_msg)

        if self.fsm_state == 4 or self.angle_trigger is True:
            #####################################################
            #	Detect the color and add color into the sequence
            ##################################################### 
            if self.find_seq:
                self.fsm_transit(5)
                time.sleep(1)

           
            if self.img is None:
                return 
            # Start detect color
            obj = self.light_buoy.position_local
            #self.img = cv2.convertScaleAbs(self.img, alpha=0.7, beta=50)
            
            color, img_mask, img_roi = self.color_detect.get_color_and_image_mask(self.img, obj.x, obj.y, obj.z, self.sim)

            print("Color = ", color)
            
            if(color != "NonDetect"): # None is black, nondetect is nothing
                #print color
                msg_img_mask = Image()
                msg_img_mask_compressed = CompressedImage()
                msg_img_mask = self.cv_bridge.cv2_to_imgmsg(img_mask, "bgr8")
                msg_img_mask_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(img_mask)
                self.pub_image_mask.publish(msg_img_mask)
                self.pub_image_mask_compressed.publish(msg_img_mask_compressed)

                msg_img_roi = Image()
                msg_img_roi_compressed = CompressedImage()
                msg_img_roi = self.cv_bridge.cv2_to_imgmsg(img_roi, "bgr8")
                msg_img_roi_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(img_roi)
                self.pub_image_roi.publish(msg_img_roi)
                self.pub_image_roi_compressed.publish(msg_img_roi_compressed)
            
            if color != "NonDetect":
            # Add color to  sequence
                self.color = color
                if not self.start_detect:
                    print "Start Detect"
                    self.t_start = rospy.get_time()
                    self.start_detect = True

                if self.tmp_color != self.color:
                    self.pre_color = self.tmp_color
                    self.tmp_color = self.color
                    self.change_color = True
                    #print "pre color = ", self.pre_color, ", tmp color = ", self.tmp_color

                self.process_seq()

            if self.start_detect and rospy.get_time() - self.t_start >= 15:
                self.guess_seq()


        if self.fsm_state == 5:
            # find most correct sequence of 4 seq
            print("4 point sequence = ", self.all_seq)
            dict_ = {}
            for seq in self.all_seq:
                dict_[seq] = self.all_seq.count(seq)
            ans = max(dict_.iteritems(), key=operator.itemgetter(1))[0]
            ans = ans + tuple("N")
            print "Ans = ", ans
            self.fsm_transit(6)
            msg = String()
            msg.data = ans[0] + " " + ans[1] + " " + ans[2]
            while(1):
                time.sleep(0.5)
                self.pub_seq.publish(msg)
                print ans


    def onShutdown(self):
        motor_msg = roboteq_drive()
        motor_msg.left = 0
        motor_msg.right = 0
        self.pub_motion.publish(motor_msg)
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('task5_node', anonymous = True)
    task5 = task5_node()
    rospy.on_shutdown(task5.onShutdown)
    rospy.spin()
