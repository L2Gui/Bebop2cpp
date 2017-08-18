cmake_minimum_required(VERSION 2.8)
project(bebop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#create libbebop2
find_package(OpenCV REQUIRED)
add_library(libbebop2 Drone.cpp)
target_link_libraries(libbebop2 ${OpenCV_LIBS})
target_link_libraries(libbebop2 m)
target_link_libraries(libbebop2 arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)

#executables
add_executable(demo demo.cpp)
target_link_libraries(demo libbebop2)

#TODO add lines for tuto1.cpp bellow

add_executable(test_kotani test_kotani.cpp)
target_link_libraries(test_kotani libbebop2)