cmake_minimum_required(VERSION 3.0.2)
project(dual_robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
