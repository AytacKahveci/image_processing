cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(Pylon QUIET)
if(NOT ${Pylon_FOUND})
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()

catkin_package(
   INCLUDE_DIRS
#  LIBRARIES pylon_camera
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Pylon_INCLUDE_DIRS}
)

add_executable(custom_pylon_node src/custom_pylon_node.cpp)
target_link_libraries(custom_pylon_node ${Pylon_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS custom_pylon_node
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
