cmake_minimum_required(VERSION 3.0.2)
project(lab_10)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obsposelist.msg
  # drone_rpy.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab_10
#  CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

