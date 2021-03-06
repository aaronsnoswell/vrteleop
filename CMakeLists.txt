cmake_minimum_required(VERSION 2.8.3)
project(vrteleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  message_generation
)

# Build our Compressed Point Cloud message type
add_message_files(
  FILES
  CompressedPointCloud2.msg
  HeaderlessPointCloud2.msg
  PolygonMesh.msg
  Polygon.msg
  VRMotionController.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)

## Grab any includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Grab MLS
add_executable(pcl_smoothing src/pcl_moving_least_squares.cpp)
target_link_libraries(pcl_smoothing ${catkin_LIBRARIES})

## Grab Compression
add_executable(pcl_compression src/pcl_compression.cpp)
add_dependencies(pcl_compression vrteleop_generate_messages_cpp)
target_link_libraries(pcl_compression ${catkin_LIBRARIES})

## Grab Decompression
add_executable(pcl_decompression src/pcl_decompression.cpp)
add_dependencies(pcl_decompression vrteleop_generate_messages_cpp)
target_link_libraries(pcl_decompression ${catkin_LIBRARIES})

## Grab Meshing
add_executable(pcl_meshing src/pcl_meshing.cpp)
add_dependencies(pcl_meshing vrteleop_generate_messages_cpp)
target_link_libraries(pcl_meshing ${catkin_LIBRARIES})

## Grab UnMeshing
add_executable(pcl_unmeshing src/pcl_unmeshing.cpp)
add_dependencies(pcl_unmeshing vrteleop_generate_messages_cpp)
target_link_libraries(pcl_unmeshing ${catkin_LIBRARIES})
