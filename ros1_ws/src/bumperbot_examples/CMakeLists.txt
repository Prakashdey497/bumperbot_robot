cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  AddTwoInts.srv
  GetTransform.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    tf2_ros
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/tf_examples_node.py
    nodes/simple_timer.py
    nodes/simple_service_server.py
    nodes/simple_service_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## For tf_examples_node.cpp

add_library(tf_examples src/tf_examples.cpp)
add_dependencies(tf_examples ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_examples ${catkin_LIBRARIES})


add_executable(tf_examples_node nodes/tf_examples_node.cpp src/tf_examples.cpp)
add_dependencies(tf_examples_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_examples_node tf_examples ${catkin_LIBRARIES})


## For simple_timer.cpp

add_executable(simple_cpp_timer nodes/simple_timer.cpp)
target_link_libraries(simple_cpp_timer ${catkin_LIBRARIES})



## For simple_service_server.cpp
add_executable(simple_cpp_service_server nodes/simple_service_server.cpp)
add_dependencies(simple_cpp_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_service_server ${catkin_LIBRARIES})


## For simple_service_client.cpp
add_executable(simple_cpp_service_client nodes/simple_service_client.cpp)
add_dependencies(simple_cpp_service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_service_client ${catkin_LIBRARIES})

