# In task6 
Download classifiers first. Please follow the [README](https://github.com/RobotX-NCTU/robotx_nctu/blob/devel-brianchuang/catkin_ws/src/dl_models/placard_classification/README.md).

## For live demo
```
$ roslaunch robotx task6_demo_zed.launch veh:=zed camera:=true rotation:=true visual:=true
```

## For offline analysis
Testing rosbags are available from [here](https://drive.google.com/drive/u/1/folders/18C7SzKAOT3L9HS4WrRwZIU-aoqQyv9ml). 
```
$ roslaunch robotx task6_demo_zed.launch veh:=zed camera:=false rotation:=true visual:=true
```
