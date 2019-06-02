cmake_minimum_required(VERSION 2.8.3)
project(teme_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
   FILES
   PoseArray.msg
)


generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teme_msgs
   CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

