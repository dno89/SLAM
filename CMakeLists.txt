project(slam)

add_definitions(-std=c++0x)

include_directories(/usr/include/eigen3/)

add_executable(slam_test src/test-main.cpp)

install(TARGETS slam_test RUNTIME DESTINATION bin)

add_subdirectory(SLAM/base)