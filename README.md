# Wire Manipulation

## Prerequisites
  - ROS Noetic Desktop
  - Gazebo 11.XX
  - gazebo_ros_pkgs

## Launching Robot Controllers

### Launch Dual Arm in Gazebo

```
roslaunch dual_robot_gazebo dual_robot_gazebo.launch
roslaunch dual_robot_moveit_config dual_robot_moveit.launch
```

### Launch Dual Arm with Real Robots

```
roslaunch dual_robot_control dual_robot_bringup.launch
```
## Launching Vision Stack

```
roslaunch vision vision.launch
```
## executing Manipulation
```
rosrun dual_robot_control clear_wire.py
```
