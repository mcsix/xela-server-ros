cmake_minimum_required(VERSION 2.8.3)
project(xela_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  SensPoint.msg
  xSensorData.msg
  xServerMsg.msg
)

add_service_files(
  FILES
  XelaSensorX.srv
  XelaSensorY.srv
  XelaSensorZ.srv
  XelaSensorXY.srv
  XelaSensorXYZ.srv
  XelaSensorStream.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/xela_server
  scripts/xela_service
  launch/service.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
