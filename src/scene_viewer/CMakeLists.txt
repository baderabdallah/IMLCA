cmake_minimum_required(VERSION 3.0.2)
project(scene_viewer)

find_package(catkin REQUIRED COMPONENTS
  lane_msgs
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/scene_viewer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
