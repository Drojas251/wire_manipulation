cmake_minimum_required(VERSION 3.0.2)
project(wire_modeling)


find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



