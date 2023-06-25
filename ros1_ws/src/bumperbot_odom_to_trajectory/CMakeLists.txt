cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_odom_to_trajectory)



find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  nodes/path_odom_plotter_nodes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


