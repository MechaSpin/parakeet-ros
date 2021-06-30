cmake_minimum_required (VERSION 3.10)
project(parakeet_ros VERSION 2.0.0)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs)

## Import Parakeet SDK
if(FIND_PKG_PARAKEET_SDK)
	set(CMAKE_FIND_PACKAGE_SORT_ORDER NATURAL)
	set(CMAKE_FIND_PACKAGE_SORT_DIRECTION DEC)
	find_package(Parakeet REQUIRED)
	set(PARAKEET_TARGET_NAME Parakeet::Parakeet)
else()
	add_subdirectory(parakeet-sdk)
	set(PARAKEET_TARGET_NAME Parakeet)
endif()

## Declare a catkin package
catkin_package()

## Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${PARAKEET_TARGET_NAME})

## Build package
add_executable(parakeet_ros_talker src/ROSNode.cpp)
target_link_libraries(parakeet_ros_talker ${catkin_LIBRARIES} ${PARAKEET_TARGET_NAME})

install( TARGETS
    parakeet_ros_talker
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
