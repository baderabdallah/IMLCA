cmake_minimum_required(VERSION 3.0.2)
project(lane_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Mlc.msg
  Pose.msg
  ScenarioData.msg
  Trajectory.msg
  VehicleInfo.msg
  EgoInfo.msg
  KPIs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
