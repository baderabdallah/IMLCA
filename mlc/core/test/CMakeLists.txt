add_executable(tests
    multiple_lane_change_test.cpp
    trajectory_change_test.cpp
)

target_link_libraries(tests
  PRIVATE
    gmock_main
    mlc
)

target_include_directories(tests PUBLIC ${CMAKE_SOURCE_DIR})

add_test(NAME tests COMMAND tests)
