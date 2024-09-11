cmake_minimum_required(VERSION 3.0.2)
project(rl_algo)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/rl_algo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)