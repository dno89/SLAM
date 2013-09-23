cmake_minimum_required(VERSION 2.0)

project(slam)

add_definitions(-std=c++0x)

include_directories(/usr/include/eigen3/)

add_executable(slam_test src/test-main.cpp)

target_link_libraries(slam_test boost_date_time)

install(TARGETS slam_test RUNTIME DESTINATION bin)

add_subdirectory(SLAM/base)