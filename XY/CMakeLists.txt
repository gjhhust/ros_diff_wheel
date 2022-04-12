cmake_minimum_required(VERSION 3.0.2)
project(XY)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES VoiceXF
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(iat_sample src/iat_online_record_sample.cpp
			                    src/linuxrec.c
		                      src/speech_recognizer.c)

target_link_libraries(
   iat_sample
   ${catkin_LIBRARIES} 
   libmsc.so -ldl -lpthread -lm -lrt -lasound
 )