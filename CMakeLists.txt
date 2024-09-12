cmake_minimum_required(VERSION 3.0.2)
project(landmark_map_builder)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  laser_geometry
  yolov5_pytorch_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl
# CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(record_landmark_node src/record_landmark_node.cpp)
add_executable(visualize_landmark_node src/visualize_landmark_node.cpp)

target_link_libraries(record_landmark_node yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(visualize_landmark_node yaml-cpp ${catkin_LIBRARIES})