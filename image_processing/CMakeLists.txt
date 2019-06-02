cmake_minimum_required(VERSION 2.8.3)
project(image_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

set(OPENCV_TRACKING_LIB "/usr/local/lib/libopencv_tracking.so")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/cmake)
message(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  roscpp
  rospy
  kuka_hw_cart_vel
  message_filters
  object_position_msgs
  dynamixel_custom_controller
  dynamic_reconfigure
  tf
  control_toolbox
  teme_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/hsvThresh.cfg
)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES image_processing
#  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs std_msgs
#  DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(videoWriter src/videoWriter.cpp)
target_link_libraries(videoWriter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pose_estimate src/pose_estimate.cpp)
target_link_libraries(pose_estimate ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENCV_TRACKING_LIB}) 
add_dependencies(pose_estimate ${PROJECT_NAME}_gencfg)

add_executable(object_positioning src/object_positioning.cpp)
target_link_libraries(object_positioning ${catkin_LIBRARIES}) 

add_executable(xy_control src/xy_control.cpp)
target_link_libraries(xy_control ${catkin_LIBRARIES})
add_dependencies(xy_control teme_msgs ${catkin_EXPORTED_TARGETS})

add_executable(yz_control src/yz_control.cpp)
target_link_libraries(yz_control ${catkin_LIBRARIES})

add_executable(xyz_control src/xyz_control.cpp)
target_link_libraries(xyz_control ${catkin_LIBRARIES})
add_dependencies(xyz_control teme_msgs ${catkin_EXPORTED_TARGETS})

add_executable(polar_coordinate src/polar_coordinate.cpp)
target_link_libraries(polar_coordinate ${catkin_LIBRARIES})
add_dependencies(polar_coordinate teme_msgs ${catkin_EXPORTED_TARGETS})

add_executable(positioning4DOF src/positioning4DOF.cpp)
target_link_libraries(positioning4DOF ${catkin_LIBRARIES})

add_executable(helis_path src/objectHelisPath.cpp)
target_link_libraries(helis_path ${catkin_LIBRARIES})
add_dependencies(helis_path object_position_msgs ${catkin_EXPORTED_TARGETS})

add_executable(rectangle_path src/objectRectanglePath.cpp)
target_link_libraries(rectangle_path ${catkin_LIBRARIES})
add_dependencies(rectangle_path object_position_msgs ${catkin_EXPORTED_TARGETS})

add_executable(polygon_path src/objectPolygonPath.cpp)
target_link_libraries(polygon_path ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENCV_TRACKING_LIB})
add_dependencies(polygon_path object_position_msgs ${catkin_EXPORTED_TARGETS})
