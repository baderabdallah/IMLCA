cmake_minimum_required(VERSION 3.0.2)
project(scenario_builder)

find_package(catkin REQUIRED COMPONENTS
  lane_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/scenario_generator_node.py src/scenario_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
