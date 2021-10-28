
FROM ros:noetic

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit  \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    python3-rosdep \
    ros-noetic-rviz \
    git-all \
    && apt-get clean

RUN apt-get -y install xauth
RUN apt-get -y install firefox

