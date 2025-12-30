cmake_minimum_required(VERSION 3.10)
project(GeoGlobe)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(GeoGlobe main.cpp)
target_link_libraries(GeoGlobe PRIVATE ${VTK_LIBRARIES})
