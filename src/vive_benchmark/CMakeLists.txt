cmake_minimum_required(VERSION 2.8)
project(vive_benchmark)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
)


include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(vive_benchmark src/vive_benchmark.cpp)
target_link_libraries(vive_benchmark ${catkin_LIBRARIES})  
