cmake_minimum_required(VERSION 2.0)

add_definitions(-std=c++0x)

add_library(slam STATIC Engine/EKFSLAMEngine.cpp DataAssociation/DataAssociation.cpp Base/core.cpp Vehicle/SimpleUnicycle.cpp Landmark/PolarPointLandmark.cpp)

target_link_libraries(slam boost_date_time)