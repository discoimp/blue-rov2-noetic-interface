cmake_minimum_required(VERSION 3.0.2)
project(rov_interface)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  mavros_msgs
  mavros
  mavlink
  libmavconn
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Add cmake target dependencies of the library
add_executable(mavlink_to_ros_node src/mavlink_to_ros_node.cpp)

target_link_libraries(mavlink_to_ros_node ${catkin_LIBRARIES})

add_dependencies(mavlink_to_ros_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)