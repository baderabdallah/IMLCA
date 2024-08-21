cmake_minimum_required(VERSION 3.0.2)
project(speed_visualizer)

find_package(catkin REQUIRED COMPONENTS
  lane_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/speed_visualizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
