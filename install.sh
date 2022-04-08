#!/usr/bin/env bash

export catkin_ws=~/wire_manipulation_framework
export repo=wire_manipulation
cd ..
mkdir -p $catkin_ws/src

mv ~/$repo $catkin_ws/src

cd ~

sudo apt-get update
sudo apt-get upgrade

cd $catkin_ws/src

git clone https://github.com/Drojas251/interbotix_ros_core.git
git clone https://github.com/Drojas251/interbotix_ros_toolboxes.git
git clone -b noetic https://github.com/Drojas251/image_pipeline.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

# moveit and ros control 
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-moveit

# python related packages 
pip install python-fcl
pip install -U scikit-learn
sudo apt-get install python-numpy python-scipy



