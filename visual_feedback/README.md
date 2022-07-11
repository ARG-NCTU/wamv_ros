# visual_feedback
Led Array for robotx competition. There are red, yellow, green, and buzzer on the visual feedback.

## How to setup
### Hardware
1. Raspberry Pi 3
2. 5VDC Relay Array

<!--![hardware setup](https://github.com/RobotX-NCTU/visual_feedback/blob/master/.imgs/hardware%20setup.jpg)-->

![Raspberry Pi 3 GPIO](https://github.com/RobotX-NCTU/visual_feedback/blob/master/.imgs/rpi3%20gpio.png)

| Rpi3  | Relay Array |
| ----- | ----------- |
| 5v    | Vcc |
| GND   | GND |
| GPIO4 | IN1 |
| GPIO17 | IN2 |
| GPIO27 | IN3 |
| GPIO22 | IN4 |


### Software
``` 
mkdir -p ~/robotx/catkin_ws/src
cd ~/robotx/catkin_ws/src
git clone https://github.com/RobotX-NCTU/visual_feedback.git
```

## catkin_make

```
source /opt/ros/kinetic/setup.bash
cd ~/robotx/catkin_ws
catkin_make
```

## How to run

```
cd ~/robotx
source /opt/ros/kinetic/setup.bash
source ~/robotx/catkin_ws/devel/setup.bash
roslaunch visual_feedback visual_feedback_node.launch

rostopic pub /visual_feedback_node/visual_fb_[red] std_msgs/Bool "data: true"
[] could be "red", "green", "yellow", "bz"
```

## Error List
### RPi.GPIO need sudo
```
sudo adduser "username" gpio
sudo reboot
```

 
