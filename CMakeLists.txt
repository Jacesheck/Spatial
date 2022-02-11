cmake_minimum_required(VERSION 3.16.3)
project(Spatial)

add_subdirectory(external/Spatial)

include_directories(external/Spatial)

add_executable(Spatial main.cpp)

target_link_libraries(Spatial PUBLIC SpatialLib)