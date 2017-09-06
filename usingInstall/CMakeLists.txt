cmake_minimum_required(VERSION 2.8)
project(testInstall)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS iostreams system filesystem log thread REQUIRED)

# All components are not needed, they were only for gnuplot demo file
#find_package(Boost COMPONENTS iostreams chrono date_time timer system filesystem log thread REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(testInstall test.cpp)
target_link_libraries(testInstall ${OpenCV_LIBRARIES})
target_link_libraries(testInstall m)
target_link_libraries(testInstall pthread)
target_link_libraries(testInstall ${Boost_LIBRARIES})
target_link_libraries(testInstall arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)
target_link_libraries(testInstall bebop2cpp)

