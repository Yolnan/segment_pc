cmake_minimum_required(VERSION 3.0.2)
project(segment_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  obj_pointcloud.msg
  obj_centroid_data.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

#atkin_package()

catkin_install_python(PROGRAMS 
  src/segment_pcl_node.py
  src/pointcloud_center.py
  src/mocap_transform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
