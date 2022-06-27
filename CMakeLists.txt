cmake_minimum_required(VERSION 3.0.2)
project(segment_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# catkin_package()

# catkin_install_python(PROGRAMS src/segment_pcl_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)