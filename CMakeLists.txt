cmake_minimum_required(VERSION 2.8)
project(bebop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)
include_directories(tuto)
include_directories(res)

#create libbebop2
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS iostreams system filesystem log thread chrono date_time timer REQUIRED)
# iostreams just for gnuplot
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

configure_file(res/calib_bd2.xml ${CMAKE_BINARY_DIR}/res/calib_bd2.xml COPYONLY)

add_library(libbebop2
        src/Drone.cpp
        src/fullnavdata.cpp
        # Just for being listed as part of the project in IDE
        include/Drone.h
        )
target_link_libraries(libbebop2 ${OpenCV_LIBRARIES})
target_link_libraries(libbebop2 m)
target_link_libraries(libbebop2 pthread)
target_link_libraries(libbebop2 ${Boost_LIBRARIES})
target_link_libraries(libbebop2 arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)

#executables
add_executable(demo demo/demo.cpp)
target_link_libraries(demo libbebop2)

add_executable(videoDisplay demo/videoDisplay.cpp)
target_link_libraries(videoDisplay libbebop2)


add_executable(fullNavData demo/fullNavData.cpp)
target_link_libraries(fullNavData libbebop2)

add_executable(test_kotani tuto/test_kotani.cpp)
target_link_libraries(test_kotani libbebop2)