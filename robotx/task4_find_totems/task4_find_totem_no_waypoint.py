#!/usr/bin/env python
import rospy

# Ros lib
from robotx_msgs.msg import ObjectPoseList
from robotx_msgs.msg import Waypoint, WaypointList
from robotx_msgs.srv import waypoint
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
import tf
import tf_conversions
import time

class Task4(object):
    def __init__(self):
        # Ros service
        self.srv_totem_cirle = rospy.Service('~clear', Trigger, self.cb_srv_clear)
        self.srv_totem_cirle = rospy.Service('~start', Trigger, self.cb_srv_start)

        self.clear_waypoint()
        self.wpt_list = []
        self.end = None
        self.start = False
        self.status = 0

        rospy.Timer(rospy.Duration(0.1), self.process)

        # Publisher
        self.pub_marker = rospy.Publisher("~way_point", MarkerArray, queue_size=1)
        self.pub_status = rospy.Publisher("/nav/arrive", Bool, queue_size=1)


        # Subscriber
        self.sub_waypoint_statud = rospy.Subscriber("/wp_nav_state", Int32, self.cb_status)
        self.sub_waypt = rospy.Subscriber("/move_base_simple/goal",  PoseStamped, self.cb_waypt) 

    def process(self, event):
        if self.start and self.end is not None and self.status == 1:
            self.end = None
            msg = Bool()
            msg.data = True
            counter = 5
            while(counter >=0):
                counter -= 1
                self.pub_status.publish(msg)

    def cb_status(self, msg):
        self.status = msg.data

    def cb_waypt(self, msg_wpt):
        wpt = msg_wpt.pose.position
        qua = msg_wpt.pose.orientation
        q = (qua.x, qua.y, qua.z, qua.w)
        euler = tf.transformations.euler_from_quaternion(q)[2]
        
        self.wpt_list.append([wpt, euler])
        self.draw_point()

    def draw_point(self):
        marker_array = MarkerArray()
        if len(self.wpt_list) is 0:
            return
        i = 0
        for wpt in self.wpt_list:
            marker = Marker()
            marker.header.frame_id = "/odom"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.id = i
            i += 1
            marker.pose.position.x = wpt[0].x
            marker.pose.position.y = wpt[0].y
            marker.pose.position.z = 0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.pub_marker.publish(marker_array)

    def cb_srv_clear(self, request):
        self.clear_waypoint()
        self.wpt_list = []
        print ("Clear all waypoints")
        return TriggerResponse()
    
    def cb_srv_start(self, request):
        print ("Start motion")
        for i in range(0, len(self.wpt_list)):
            wpt = self.wpt_list[i][0]
            euler = self.wpt_list[i][1]
            self.add_waypoint(wpt.x, wpt.y, euler)
            
        self.end = self.wpt_list[len(self.wpt_list)-1]
    
        self.start_waypoint()
        self.start = True

        time.sleep(3)

        return TriggerResponse()

    def add_waypoint(self, x, y, yaw):
        rospy.wait_for_service("/add_waypoint")
        try:
            send_waypoint = rospy.ServiceProxy("/add_waypoint", waypoint)
            waypoint_len = send_waypoint(x, y, yaw, 0)
            print "set way_point, x = ", x, ", y = ", y, ", yaw = ", yaw

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def start_waypoint(self):
        rospy.wait_for_service("/start_waypoint_nav")
        try:
            start_waypoint = rospy.ServiceProxy("/start_waypoint_nav", Trigger)
            start_ = start_waypoint()
 
        except rospy.ServiceException, e:
            print "Service call fa  iled: %s"%e        

    def clear_waypoint(self):
        rospy.wait_for_service("/clear_waypoints")
        try:
            clear_waypoint = rospy.ServiceProxy("/clear_waypoints", Trigger)
            clear_ = clear_waypoint()
            print "clear waypoints"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e            

    def onShutdown(self):
        rospy.loginfo("Node shutdown")

if __name__ == '__main__':
    rospy.init_node('task4_node', anonymous = True)
    task4 = Task4()
    rospy.on_shutdown(task4.onShutdown)
    rospy.spin()
