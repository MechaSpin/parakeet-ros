cmake_minimum_required (VERSION 3.5)
project(parakeet_ros VERSION 3.0.0)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

## Import Parakeet SDK
if(FIND_PKG_PARAKEET_SDK)
	set(CMAKE_FIND_PACKAGE_SORT_ORDER NATURAL)
	set(CMAKE_FIND_PACKAGE_SORT_DIRECTION DEC)
	find_package(Parakeet REQUIRED)
else()
	add_subdirectory(parakeet-sdk)
endif()

## Declare a catkin package
catkin_package()

## Build package
add_executable(parakeet_ros_talker src/main.cpp src/ROSNode.cpp include/ROSNode.h)
target_include_directories(parakeet_ros_talker PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(parakeet_ros_talker ${catkin_LIBRARIES} Parakeet::Parakeet)

## Install
install(TARGETS
    parakeet_ros_talker
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
