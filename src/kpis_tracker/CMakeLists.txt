cmake_minimum_required(VERSION 3.0.2)
project(kpis_tracker)

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

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(src/test_kpis_tracker_node.py)
  
  add_custom_target(pytest 
    COMMAND pytest -v ${CMAKE_CURRENT_SOURCE_DIR}/kpis_tracker
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kpis_tracker
  )
endif()

catkin_install_python(PROGRAMS src/kpis_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
