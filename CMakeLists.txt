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
  message_generation
)

add_message_files(
  FILES
  Landmark.msg
  Landmarks.msg
  LandmarkPose.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(calc_landmark_node src/calc_landmark_node.cpp)
add_executable(dbscan src/dbscan.cpp)
add_executable(edit_landmark_node src/edit_landmark_node.cpp)

set_target_properties(dbscan PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/)

target_link_libraries(calc_landmark_node yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(dbscan yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(edit_landmark_node yaml-cpp ${catkin_LIBRARIES})