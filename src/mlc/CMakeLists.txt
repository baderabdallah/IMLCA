cmake_minimum_required(VERSION 3.0.2)
project(mlc)

find_package(catkin REQUIRED COMPONENTS
  lane_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS lane_msgs roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/core
  src
)

include_directories(src/core)
include_directories(src)

add_executable(mlc_node
    src/mlc_node.cpp
    src/core/follow_lane_trajectory_planner.cpp
    src/core/lane_change_trajectory_planner.cpp
    src/core/multiple_lane_change.cpp
    src/core/utils.cpp
)

target_link_libraries(mlc_node PUBLIC ${catkin_LIBRARIES})
