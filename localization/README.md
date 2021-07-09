# localization

Using Vector V103 GPS and Microstrain 3DM-GX5-25 IMU to localize WAM-V

### To make this work space, please make sure you have libgps and ROS GPS package
```
  $ sudo apt-get install libgps-dev
  $ sudo apt-get install ros-kinetic-gps-* ros-kinetic-gpsd-client
```
### To run robot-localization package, please type
```
  $ sudo apt-get install ros-kinetic-robot-localization
```

### To make the work space, please type
```
  $ cd ~/robotx_sensor/catkin_ws
  $ catkin_make install
```

### Make sure you can import utm in python
```
  $ pip install utm
```

## How to run
### Real-robot only
First, we have to give the permission for USB ports
```
  $ source ~/robotx_nctu/set_sensors_port.sh
```
And then turn on the sensors
```
  $ roslaunch localization sensors.launch
```
Next, for real world
```
  $ roslaunch localization gps_imu_localization.launch
```
For Gazebo, run
```
  $ roslaunch localization gps_imu_localization_gazebo.launch
```
## Topics flow, service and frame definition

1. Input topics:  
- /imu/data: [sensor_msgs/Imu](http://docs.ros.org/lunar/api/sensor_msgs/html/msg/Imu.html)  
- /fix: [sensor_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)  
- /vel: [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)  

#### Make sure the covariance of each message is non-zero matrix or the filter may break

2. Output topics:  
- /odometry/filtered: [nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)  
    This is the major output  
Since the orientation is represented in quaternion, use the block below to convert it to Euler angle

```
	q = [msg.pose.pose.orientation.x, \
	     msg.pose.pose.orientation.y, \
	     msg.pose.pose.orientation.z, \
	     msg.pose.pose.orientation.w]
	_, _, yaw = tf.transformations.euler_from_quaternion(q)
```
- /gps/filtered: [sensor_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)  
    You can visualize the filtered GPS data with tools such as Google Map  

3. Service:  
- /set_wps_from_lonlat  
  - request: latitude and longitude  
  - response: Coordinate w.r.t odom frame, the server will publish a topic /waypoints_lonlat with message type  
[geometry_msgs/PoseArray](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseArray.html) and also broadcast an TF
  - Example: rosservice call /set_wps_from_lonlat "lat: 24.0 lon: 121.0"
- /clear_path  
  - request: clear in bool
  - response: if clear is true, clear the path and return with no response
  - Example: rosservice call /clear_path "clear: true"
- /clear_gps_path
  - same as above but for gps odom path (red one)
4. Frames:  
TF tree: utm -> odom -> base_link -> sensors...  
- "utm" is a frame where zero latitude and zero longitude as origin, East (with azimuth 90 degrees) as positive x-axis and North (with azimuth 0 degree) as positive y-axis  
- "odom" is a frame which used the position where we start the EKF node as origin, East (with azimuth 90 degrees) as positive x-axis and North (with azimuth 0 degree) as positive y-axis  
