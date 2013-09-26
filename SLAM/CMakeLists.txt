cmake_minimum_required(VERSION 2.0)

add_definitions(-std=c++0x)

add_library(slam STATIC Base/core.cpp Engine/EKFSLAMEngine.cpp DataAssociation/DataAssociation.cpp)

target_link_libraries(slam boost_date_time)