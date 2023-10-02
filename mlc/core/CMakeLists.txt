add_library(mlc
    collision_detector.cpp
    follow_lane_trajectory_planner.cpp
    lane_change_trajectory_planner.cpp
    multiple_lane_change.cpp
    utils.cpp
)

add_library(${PROJECT_NAME}::mlc ALIAS mlc)

target_include_directories(mlc PUBLIC ${CMAKE_SOURCE_DIR})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()