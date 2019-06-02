cmake_minimum_required(VERSION 2.8.3)
project(object_position_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)


add_message_files(
   FILES
   ObjectPositionMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime roscpp std_msgs
)
