
FROM ros:noetic

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y 

ENV DEBAIN_FRONTEND = noninteractive

RUN apt-get install -y \
    ros-noetic-moveit  \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-ros-numpy \
    ros-noetic-gazebo-ros \
    ros-noetic-cv-bridge \
    ros-noetic-depth-image-proc \
    ros-noetic-realsense2-camera \
    ros-noetic-pcl-ros \ 
    python3-rosdep \
    xauth \
    git-all 


###
RUN apt-get -y install pip
RUN pip install python-fcl numpy scipy -U scikit-learn

WORKDIR /root/wire_ws/src
RUN git clone https://github.com/Drojas251/interbotix_ros_toolboxes.git \
  && git clone https://github.com/Drojas251/interbotix_ros_core.git \
  && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
  && git clone https://github.com/Drojas251/wire_manipulation.git \
  && git clone -b noetic https://github.com/Drojas251/interbotix_ros_manipulators


WORKDIR /root/wire_ws
RUN apt-get update -y \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make 

