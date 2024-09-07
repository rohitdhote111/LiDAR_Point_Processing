cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)

target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

install(TARGETS pointcloud_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

