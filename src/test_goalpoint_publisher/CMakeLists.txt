cmake_minimum_required(VERSION 2.8.3)
project(test_goalpoint_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  roslib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_goalpoint_publisher_node src/test_goalpoint_publisher_node.cpp)
target_link_libraries(test_goalpoint_publisher_node ${catkin_LIBRARIES})

