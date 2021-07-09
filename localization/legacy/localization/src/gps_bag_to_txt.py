#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import NavSatFix
from geodesy.utm import UTMPoint, fromLatLong

def cb_fix(msg):
    global counter, origin_x, origin_y
    utm_point = fromLatLong(msg.latitude, msg.longitude)

    print "Save data " + str(counter)
    counter += 1
    f.write(str(msg.latitude))
    f.write(' ')
    f.write(str(msg.longitude))

    if origin_x is -1 and origin_y is -1:
        origin_x = utm_point.easting
        origin_y = utm_point.northing

        f.write(' ')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))   
        
    else:
        f.write(' ')
        f.write(str(utm_point.easting - origin_x))
        f.write(' ')
        f.write(str(utm_point.northing - origin_y))  

    f.write('\n')

def shutdown():
    print "gps_bag_to_txt node shutdown..."
    rospy.delete_param('~file_name')
    f.close()

if __name__ == '__main__':
    rospy.init_node('gps_bag_to_txt')
    
    counter = 0
    origin_x = -1
    origin_y = -1

    rospy.Subscriber('~fix', NavSatFix, cb_fix, queue_size = 10)
    rospack = rospkg.RosPack()
    
    file_name = rospy.get_param('~file_name')
    root_folder = rospack.get_path('localization')
    
    f = open(root_folder + '/map_plot/' + file_name + '.txt', 'w')

    print "save file path = ", root_folder + '/map_plot/' + file_name + '.txt'

    rospy.on_shutdown(shutdown)
    rospy.spin()
