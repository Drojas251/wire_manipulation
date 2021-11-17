cmake_minimum_required(VERSION 3.0.2)
project(wire_modeling_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
)

add_service_files(
  FILES
  ProcessPointCloud.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



