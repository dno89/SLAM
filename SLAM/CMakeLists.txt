cmake_minimum_required(VERSION 2.0)

add_definitions(-std=c++0x -DEKFSLAM_ENABLE_TEST)

add_library(slam STATIC Engine/EKFSLAMEngine.cpp DataAssociation/DataAssociation.cpp DataAssociation/hungarian.c Base/core.cpp Vehicle/SimpleUnicycle.cpp Landmark/PolarPointLandmark.cpp Landmark/CartesianPointLandmark.cpp Landmark/PolarLineLandmark.cpp)

target_link_libraries(slam boost_date_time)