cmake_minimum_required(VERSION 2.8.3)
project(test_goalpoint_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  roslib
)
find_package(nlohmann_json REQUIRED)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
find_package(Boost 1.65.0 REQUIRED COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(test_goalpoint_publisher_node src/test_goalpoint_publisher_node.cpp)
target_link_libraries(test_goalpoint_publisher_node ${catkin_LIBRARIES} nlohmann_json::nlohmann_json ${Boost_LIBRARIES})

