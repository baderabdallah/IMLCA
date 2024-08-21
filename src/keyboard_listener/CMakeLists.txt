cmake_minimum_required(VERSION 3.0.2)
project(keyboard_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(src)

add_executable(keyboard_listener 
    src/keyboard_listener.cpp
)

target_link_libraries(keyboard_listener
 PUBLIC ${catkin_LIBRARIES}
)
