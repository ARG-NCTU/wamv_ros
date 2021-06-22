#!/usr/bin/env python
'''                                                                            
Author: Tony Hsiao                                                              
Date: 2018/06/13  
Last update: 2018/07/21                                          
Placard Detection by using caffe
    Input:  Mser region proposal images
    Output: Image combined with prediction results
            Prediction result msg                                                            
''' 
import rospy
from std_msgs.msg import Bool, String, Int32
from robotx_msgs.msg import RegionClassification, RegionProposal, RegionProposalsImage, roboteq_drive, UsvDrive, WheelsCmdStamped, ObjectPose, ObjectPoseList
import time
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2
import numpy as np
from os.path import expanduser
caffe_root = expanduser("~")
sys.path.insert(0, caffe_root + '/caffe/python')
import caffe
import os
import heapq
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import math


class Task6CenterNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
 
        #self.save_result = rospy.get_param('~save_result', False)
        #self.top1_threshold = rospy.get_param('~top1_threshold', 0.9)  
        #self.diff_threshold = rospy.get_param('~diff_threshold', 1000)

        self.model_name = rospy.get_param('~model_name')
        self.target_placard = rospy.get_param('~target_placard', "")
        self.checking_number = rospy.get_param('~checking_number', 100)
        self.checking_number = int(self.checking_number)
        self.gazebo = rospy.get_param('~gazebo', False) 
        self.task7= rospy.get_param('~task7', False)
        rospy.loginfo('[%s] model name = %s' %(self.node_name, self.model_name))
        rospack = rospkg.RosPack()
        self.model_Base_Dir = rospack.get_path('dl_models') + '/placard_classification/' + self.model_name + '/'
        self.labels = []
        self.placard_vote = []
        self.state = False
        self.get_motion_left = True
        self.get_motion_mid = False  
        self.get_motion_right = True
        self.motion_sum = np.zeros(3)
        self.motion_legal = 0
        with open(self.model_Base_Dir+'lab_list.txt', 'r') as f:
            lines = f.readlines()
        for line in lines:
            line = line.replace('\n', '')
            self.labels.append(line)
            self.placard_vote.append(0)
        print self.labels

        #Publisher
        # placard classification
        self.sub_obj_list = rospy.Subscriber("/obj_list/map", ObjectPoseList, self.cb_obj, queue_size=1)

        self.pub_placard_result = rospy.Publisher("/task6_placard", String, queue_size=1)
        self.pub_depart = rospy.Publisher("/task6/depart", Bool, queue_size=1)
        self.pub_classification_switch_mid = rospy.Publisher("~prediction_switch_mid", Bool, queue_size=1)    
        self.pub_classification_switch_left = rospy.Publisher("~prediction_switch_left", Bool, queue_size=1) 
        self.pub_classification_switch_right = rospy.Publisher("~prediction_switch_right", Bool, queue_size=1) 
        self.pub_ete_classification_switch_mid = rospy.Publisher("~ete_prediction_switch_mid", Bool, queue_size=1)    
        self.pub_ete_classification_switch_left = rospy.Publisher("~ete_prediction_switch_left", Bool, queue_size=1) 
        self.pub_ete_classification_switch_right = rospy.Publisher("~ete_prediction_switch_right", Bool, queue_size=1) 
        #Publish WAM-V command in gazebo 
        if self.gazebo:
            self.pub_wamv_drive = rospy.Publisher("/cmd_drive", UsvDrive, queue_size=1)
        else :
            self.pub_wamv_drive = rospy.Publisher("/cmd_drive", roboteq_drive, queue_size=1) 
        self.pub_wheels_cmd = rospy.Publisher("/mecanum/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        #Subscriber   
        self.sub_arrive = rospy.Subscriber("/task6/arrive", Bool, self.cbArrive, queue_size=1)
        self.sub_classifiacation_result_mid = rospy.Subscriber("~classification_result_mid", RegionClassification, self.cbResultmid, queue_size=1)
        self.sub_classifiacation_result_left = rospy.Subscriber("~classification_result_left", RegionClassification, self.cbResultleft, queue_size=1)
        self.sub_classifiacation_result_right = rospy.Subscriber("~classification_result_right", RegionClassification, self.cbResultright, queue_size=1)  
        self.sub_ete_classifiacation_result_mid = rospy.Subscriber("~ete_classification_result_mid", RegionClassification, self.cbEteresultmid, queue_size=1)
        self.sub_ete_classifiacation_result_left = rospy.Subscriber("~ete_classification_result_left", RegionClassification, self.cbEteresultleft, queue_size=1)
        self.sub_ete_classifiacation_result_right = rospy.Subscriber("~ete_classification_result_right", RegionClassification, self.cbEteresultright, queue_size=1)

        #self.pub_deliver(1, 0, 0, 0)
        self.sub_mode = rospy.Subscriber('/mode', Int32, self.cb_mode)
    
    def cb_mode(self, msg):
        if msg.data == 1:
            print "wait Shooting"
            #rospy.sleep(3)
            #print 'shooting'
            #self.pub_deliver(-1, 1, 1, 0)
            #self.pub_deliver(-1, 1, 1, 0)
            #self.pub_deliver(-1, 1, 1, 0)
            #self.pub_deliver(-1, 1, 1, 0)
            #self.pub_deliver(-1, 1, 1, 0)

    def cb_obj(self, msg):
        for obj in msg.list:
            if obj.type == "dock":
                x = obj.position_local.x
                y = obj.position_local.y
                dis = math.sqrt(x*x+y*y)
                print 'dock distance: ', dis
                if dis <= 6:
                    self.prediction_state_changeing(False, False)
                    motion_msg = roboteq_drive()
                    motion_msg.left = 0
                    motion_msg.right = 0
                    print 'stop'
                    print 'motor left: ', motion_msg.left
                    print 'motor right: ', motion_msg.right
                    self.pub_wamv_drive.publish(motion_msg)
                    rospy.sleep(20)
                    motion_msg.left = -800
                    motion_msg.right = -800
                    print "back" 
                    print 'motor left: ', motion_msg.left
                    print 'motor right: ', motion_msg.right
                    self.pub_wamv_drive.publish(motion_msg)
                    rospy.sleep(10)

                    


    def placard_checking(self):
        if max(self.placard_vote) >= self.checking_number:
            max_index = self.placard_vote.index(max(self.placard_vote))
            print "get placard: ", str(self.labels[max_index])
            print "the target placard:", self.target_placard

            if str(self.labels[max_index]) == str(self.target_placard):
                print "get correct placard: ", str(self.labels[max_index]) 
                print 'stop classification keep motion'
                self.prediction_state_changeing(False, True)
                placard_report_msg = String()
                placard_report_msg.data = str(self.labels[max_index])
                self.pub_placard_result.publish(placard_report_msg)

            else:
                print "get wrong placard: ", str(self.labels[max_index])
                print "close prediction----------------"
                self.prediction_state_changeing(False, False)
                state_msg = Bool()
                state_msg.data = True
                self.pub_depart.publish(state_msg)

            for i in range(len(self.placard_vote)):
                self.placard_vote[i] = 0
            print 'clear placard vote'

    def prediction_state_changeing(self, classification_state, motion_state):
        state_msg = Bool()
        state_msg.data = classification_state
        self.pub_classification_switch_right.publish(state_msg)
        self.pub_classification_switch_left.publish(state_msg)
        self.pub_classification_switch_mid.publish(state_msg)
        state_msg.data = motion_state
        self.pub_ete_classification_switch_right.publish(state_msg)
        self.pub_ete_classification_switch_left.publish(state_msg)
        self.pub_ete_classification_switch_mid.publish(state_msg)  

    def cbArrive(self, switch_msg):
        if self.state == False and switch_msg.data == True:  
            print 'arrive testing point and start predction'
            #self.clear_waypoint()
            #self.pub_deliver(-1, 1, 0, 0)
            #self.prediction_state_changeing(False, True)
            self.prediction_state_changeing(True, True)
        self.state = switch_msg.data

    def clear_waypoint(self):
        rospy.wait_for_service("/clear_waypoints")
        try:
            clear_waypoint = rospy.ServiceProxy("/clear_waypoints", Trigger)
            clear_ = clear_waypoint()
            print "clear waypoints"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e    
 
    def cbResultmid(self, prediction_msg):
        max_index = prediction_msg.probs.index(max(prediction_msg.probs))
        self.placard_vote[max_index] += 1 
        self.placard_checking()
        print self.placard_vote

    def cbResultleft(self, prediction_msg):
        max_index = prediction_msg.probs.index(max(prediction_msg.probs))
        self.placard_vote[max_index] += 1 
        self.placard_checking()
        print self.placard_vote

    def cbResultright(self, prediction_msg):
        max_index = prediction_msg.probs.index(max(prediction_msg.probs))
        self.placard_vote[max_index] += 1 
        self.placard_checking()
        print self.placard_vote

    def pub_motion(self, output_prob):
        motion_gain = 300
        motion_base = 600
        motion_diff = output_prob[2] - output_prob[0]
        if self.gazebo:
            motion_msg = UsvDrive()
            motion_msg.left = (motion_base + (motion_gain * motion_diff))* 0.001
            motion_msg.right = (motion_base - (motion_gain * motion_diff)) * 0.001
            #print 'gazebo'
            print 'motor left: ', motion_msg.right
            print 'motor right: ', motion_msg.left
        else:
            motion_msg = roboteq_drive()
            motion_msg.left = (motion_base - (motion_gain) * motion_diff) 
            motion_msg.right = (motion_base + (motion_gain) * motion_diff) 
            print "real-world: ", motion_msg.right, '  ', motion_msg.left
            #print 'motor left: ', motion_msg.left
            #print 'motor right: ', motion_msg.right
        self.pub_wamv_drive.publish(motion_msg)

    def pub_deliver(self, rf, lf, rr, lr):
        #u_r_f_limited = joy_msg.buttons[3] + joy_msg.buttons[0]
        #u_l_f_limited = joy_msg.buttons[2] + joy_msg.buttons[1]
        #u_r_r_limited = joy_msg.buttons[4] + joy_msg.buttons[5]
        #u_l_r_limited = joy_msg.buttons[6] + joy_msg.buttons[7]

        u_r_f_limited = rf
        u_l_f_limited = lf
        u_r_r_limited = rr
        u_l_r_limited = lr

        msg_wheels_cmd = WheelsCmdStamped()
        #msg_wheels_cmd.header.stamp =joy_msg.header.stamp

        msg_wheels_cmd.vel_right_front = u_r_f_limited
        msg_wheels_cmd.vel_left_front = u_l_f_limited
        msg_wheels_cmd.vel_right_rear = u_r_r_limited
        msg_wheels_cmd.vel_left_rear = u_l_r_limited

        msg_wheels_cmd.theta = 1
        print 'launch: ', ' ', u_r_f_limited, ' ',  u_l_f_limited, ' ', u_r_r_limited, ' ', u_l_r_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)
        print 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'

    def excute_motion(self):
        if self.get_motion_mid == True and self.get_motion_left == True and self.get_motion_right == True:
            print 'motion_sum: ', self.motion_sum
            motion_gain = 500
            motion_base = 300
            #motion_diff = self.motion_sum[0] / self.motion_legal - self.motion_sum[2] / self.motion_legal 
            motion_diff = self.motion_sum[0] + self.motion_sum[1] + self.motion_sum[2] - self.motion_sum[4] - self.motion_sum[5] - self.motion_sum[6]
            #motion_diff = self.motion_sum[0]- self.motion_sum[2]
            if self.gazebo:
                motion_msg = UsvDrive()
                motion_msg.left = (motion_base + (motion_gain * motion_diff))* 0.001
                motion_msg.right = (motion_base - (motion_gain * motion_diff)) * 0.001
                #print 'gazebo'
                #print 'motor left: ', motion_msg.left
                #print 'motor right: ', motion_msg.right
            else:
                motion_msg = roboteq_drive()
                motion_msg.left = (motion_base - (motion_gain) * motion_diff) 
                motion_msg.right = (motion_base + (motion_gain) * motion_diff) 
                print "real-world"
                print 'motor left: ', motion_msg.left
                print 'motor right: ', motion_msg.right
            self.pub_wamv_drive.publish(motion_msg)
            self.get_motion_mid = False
            self.get_motion_left = False
            self.get_motion_right = False
            self.motion_legal = 0
            self.motion_sum = np.zeros(3)

    def cbEteresultmid(self, prediction_msg):
        #self.excute_motion()
        #if self.get_motion_mid == False:
        #    self.motion_sum[0] += prediction_msg.probs[0]
        #    self.motion_sum[1] += prediction_msg.probs[1]
        #    self.motion_sum[2] += prediction_msg.probs[2]
        #    self.get_motion_mid = True
        #    self.motion_legal +=1
            #print 'motion_sum: ', self.motion_sum

        self.pub_motion(prediction_msg.probs)
        #return
        #self.image_switch = switch_msg.data
        #if self.image_switch:
        #    print 'turn on prediction'
        #else:
        #    print 'turn off'

    def cbEteresultleft(self, prediction_msg):
        self.excute_motion()
        if self.get_motion_left == False:
            self.motion_sum[0] += 0
            self.motion_sum[1] += prediction_msg.probs[0]
            self.motion_sum[2] += ((prediction_msg.probs[1] + prediction_msg.probs[2]) * 0.5)
            #self.motion_sum[0] += prediction_msg.probs[0]
            #self.motion_sum[1] += prediction_msg.probs[1]
            #self.motion_sum[2] += prediction_msg.probs[2]
            self.get_motion_left = True
            self.motion_legal +=1
            print 'motion_sum: ', self.motion_sum

    def cbEteresultright(self, prediction_msg):
        self.excute_motion()
        if self.get_motion_right == False:
            self.motion_sum[0] += ((prediction_msg.probs[1] + prediction_msg.probs[0]) * 0.5)
            self.motion_sum[1] += prediction_msg.probs[2]
            self.motion_sum[2] += 0
            #self.motion_sum[0] += prediction_msg.probs[0]
            #self.motion_sum[1] += prediction_msg.probs[1]
            #self.motion_sum[2] += prediction_msg.probs[2]
            self.get_motion_right = True
            self.motion_legal +=1
            print 'motion_sum: ', self.motion_sum


    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown " %(self.node_name))

if __name__ == '__main__':
        rospy.init_node('task6_center_node',anonymous=False)
        task6_center_node = Task6CenterNode()
        rospy.on_shutdown(task6_center_node.onShutdown)
        rospy.spin() 
