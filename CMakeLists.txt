## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8.3)
project(savari)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs savari_msgs)
catkin_package()

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs 
  INCLUDE_DIRS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${savari_msgs_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(savari_ros src/savari_ros.cpp)
target_link_libraries(savari_ros ${catkin_LIBRARIES})


#############
## Testing ##
#############
# Savari OBU application (client) must be running before starting this ROS node.
# New Terminal:
#			$ roscore
# New Terminal:		
#			$ source /catkin_ws/devel/setup.bash
#			$ rosrun savari savari_ros
# New Terminal:		
#			$ source /catkin_ws/devel/setup.bash
#			$ rostopic list
