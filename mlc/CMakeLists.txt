cmake_minimum_required(VERSION 3.16)

project(mlc_project LANGUAGES CXX)

add_compile_options(-std=c++17 -Wall -Wextra -pedantic -Werror)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Manage gtest dependency
option(BUILD_TESTING "Testing enabled." On)
if (BUILD_TESTING)

    # Unpack googletest library at configure time.
    configure_file(./CMakeLists.txt.googletest googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
    if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker settings on Windows.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/googletest
                     ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                     EXCLUDE_FROM_ALL)

    enable_testing()
endif()

add_subdirectory(core)

