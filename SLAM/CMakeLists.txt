cmake_minimum_required(VERSION 2.0)

add_definitions(-std=c++0x)

add_library(slam STATIC DataAssociation/BasicDataAssociation.cpp Engine/EKFSLAMEngine.cpp Base/core.cpp)

target_link_libraries(slam boost_date_time)