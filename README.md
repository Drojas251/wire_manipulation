# Wire Manipulation

## Required packages 
### source install
```
https://github.com/Drojas251/interbotix_ros_toolboxes.git
https://github.com/Drojas251/interbotix_ros_core.git
https://github.com/ROBOTIS-GIT/DynamixelSDK.git

```

### Other packages 
```
pip install python-fcl
scipy
sklearn
sudo apt-get install ros-noetic-realsense2-camera
```
## ROS Version and Dependencies 
  - ROS Noetic Desktop
  - Moveit
  - ros_control
  - gazebo_ros_control

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
