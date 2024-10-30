cmake_minimum_required(VERSION 3.0.2)
project(landmark_map_builder)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf2
  tf2_ros
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  laser_geometry
  yolov5_pytorch_ros
  interactive_markers
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(record_landmark_node src/record_landmark_node.cpp)
add_executable(dbscan src/dbscan.cpp)
add_executable(visualize_landmark_node src/visualize_landmark_node.cpp)

set_target_properties(dbscan PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/)

target_link_libraries(record_landmark_node yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(dbscan yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(visualize_landmark_node yaml-cpp ${catkin_LIBRARIES})