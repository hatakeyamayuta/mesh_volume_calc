cmake_minimum_required(VERSION 3.5)
project(mesh_vol)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-o3")

find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(mesh_vol ./src/main.cpp)
target_link_libraries (mesh_vol ${PCL_LIBRARIES})
