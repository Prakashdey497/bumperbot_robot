cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  sensor_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/imu_republisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(imu_republisher_node nodes/imu_republisher_node.cpp)
target_link_libraries(imu_republisher_node ${catkin_LIBRARIES})