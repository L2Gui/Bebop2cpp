cmake_minimum_required(VERSION 2.8)
project(bebop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_executable(bebop main.cpp)
target_link_libraries(bebop arsal ardiscovery arcontroller curses)

add_library(libbebop2 Drone.cpp)
target_link_libraries(libbebop2 arsal ardiscovery arcontroller)

add_executable(demo demo.cpp)
target_link_libraries(demo libbebop2)