# Wire Manipulation

## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Intel Realsense d435i
- Two ViperX 300s 6DOF arms

## Docker Install
```
mkdir -p wire_manip_ws/src
cd ~/wire_manip_ws/src
git clone https://github.com/Drojas251/wire_manipulation.git
cd wire_manipulation/Docker
docker build -t wire_manip/env:latest
docker run -it --net=host -v /tmp/.X11-unix -v /dev:/dev -e DISPLAY --privileged wire_manip/env bash

```
### Local Source Install
```
mkdir -p wire_manip_ws/src
cd ~/wire_manip_ws/src

sudo apt-get update
rosdep update

git clone https://github.com/Drojas251/wire_manipulation.git
git clone https://github.com/Drojas251/interbotix_ros_toolboxes.git
git clone https://github.com/Drojas251/interbotix_ros_core.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic https://github.com/Drojas251/interbotix_ros_manipulators

pip install python-fcl
sudo apt-get install python-numpy python-scipy
pip install -U scikit-learn
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install -y ros-noetic-ros-noetic-moveit
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make

```


## Launching Demo

### Launch Dual Arm in Gazebo

```
roslaunch dual_robot_bringup bringup.launch use_sim:=true
```

### Launch Dual Arm with Real Robots

```
roslaunch dual_robot_bringup bringup.launch use_actual:=true
```

## Execute demo
```
rosrun dual_robot_control clear_wire_executor.py
```
