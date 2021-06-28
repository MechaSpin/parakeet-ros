cmake_minimum_required (VERSION 3.10)
project(parakeet_ros VERSION 1.0.2)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs)

## Import Parakeet SDK
find_package(Parakeet REQUIRED)

## Declare a catkin package
catkin_package()

## Include directories
include_directories(include ${catkin_INCLUDE_DIRS} Parakeet::Parakeet)

## Build package
add_executable(parakeet_ros_talker src/ROSNode.cpp)
target_link_libraries(parakeet_ros_talker ${catkin_LIBRARIES} Parakeet::Parakeet)
