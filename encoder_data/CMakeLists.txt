cmake_minimum_required(VERSION 2.8.3)
project(encoder_data)

## Compile as C++11, supported in ROS Kinetic and newer
  add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   encoderval.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
  std_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

  add_executable(encoder_node  src/encoder_QSB.cpp)


## Specify libraries to link a library or executable target against
  target_link_libraries(encoder_node
    ${catkin_LIBRARIES}
  )