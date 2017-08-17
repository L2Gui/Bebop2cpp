cmake_minimum_required(VERSION 2.8)
project(bebop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_executable(bebop main.cpp)
target_link_libraries(bebop arsal ardiscovery arcontroller curses)



find_package(OpenCV REQUIRED)
add_library(libbebop2 Drone.cpp)
target_link_libraries(libbebop2 ${OpenCV_LIBS})
target_link_libraries(libbebop2 m)
target_link_libraries(libbebop2 arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)

add_executable(demo demo.cpp)
target_link_libraries(demo libbebop2)

add_executable(streamDemo streamDemo.cpp)
target_link_libraries(streamDemo arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)
target_link_libraries(streamDemo ${OpenCV_LIBS})

#TODO add lines for tuto1.cpp bellow

