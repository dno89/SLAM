cmake_minimum_required(VERSION 2.0)

project(slam)

add_definitions(-std=c++0x)

include_directories(/usr/include/eigen3/)
include_directories(include/)

# add_subdirectory(SLAM)
#cmake_minimum_required(VERSION 2.0)
#add_definitions(-std=c++0x -DEKFSLAM_ENABLE_TEST)

add_library(slam STATIC src/SLAM/Engine/EKFSLAMEngine.cpp src/SLAM/DataAssociation/DataAssociation.cpp src/SLAM/DataAssociation/hungarian.c src/SLAM/Base/core.cpp src/SLAM/Vehicle/SimpleUnicycle.cpp src/SLAM/Landmark/PolarPointLandmark.cpp src/SLAM/Landmark/CartesianPointLandmark.cpp src/SLAM/Landmark/PolarLineLandmark.cpp src/SLAM/ProprioceptiveSensor/IntegratedOdometry.cpp)

target_link_libraries(slam boost_date_time)



## TEST PROGRAM ##
add_executable(slam_test src/test-main.cpp src/SLAM/DataAssociation/hungarian.c)
target_link_libraries(slam_test slam)
install(TARGETS slam_test RUNTIME DESTINATION bin)
