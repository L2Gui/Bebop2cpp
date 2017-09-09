cmake_minimum_required(VERSION 2.8)
project(bebop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)
include_directories(tuto)
include_directories(res)

#create libbebop2cpp
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS iostreams chrono date_time timer system filesystem log thread REQUIRED)
find_package(Eigen3 REQUIRED)

# Needed for the Gnuplots examples
#find_package(Boost COMPONENTS REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

#Copying the calibration file
configure_file(res/calib_bd2.xml ${CMAKE_BINARY_DIR}/res/calib_bd2.xml COPYONLY)

file(GLOB_RECURSE SRC_FILES
        ${PROJECT_SOURCE_DIR}/src/*.h
        ${PROJECT_SOURCE_DIR}/src/*.hpp
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        )

file(GLOB_RECURSE INCLUDE_FILES
        ${PROJECT_SOURCE_DIR}/include/*.h
        ${PROJECT_SOURCE_DIR}/include/*.hpp
        )

add_library(bebop2cpp SHARED
        ${SRC_FILES}
        ${INCLUDE_FILES}
        )

include_directories(${PROJECT_SOURCE_DIR}/include)

target_link_libraries(bebop2cpp ${OpenCV_LIBRARIES})
target_link_libraries(bebop2cpp m)
target_link_libraries(bebop2cpp pthread)
target_link_libraries(bebop2cpp ${Boost_LIBRARIES})
target_link_libraries(bebop2cpp arsal ardiscovery arcontroller arnetwork arcommands arnetworkal)

## install
#install(TARGETS bebop2cpp
#        EXPORT bebop2cpp
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin
#        PUBLIC_HEADER DESTINATION include)

#install(EXPORT)
#install(DIRECTORY include/ PUBLIC_HEADER DESTINATION include/bebop2cpp
#        FILES_MATCHING PATTERN "*.h")

#install(DIRECTORY src/ PRIVATE_HEADER DESTINATION include/bebop2cpp
#        FILES_MATCHING PATTERN "*.h")


install(TARGETS bebop2cpp LIBRARY DESTINATION lib)
install(FILES ${INCLUDE_FILES} DESTINATION include/bebop2cpp/)




#executables
add_executable(demo demo/demo.cpp)
target_link_libraries(demo bebop2cpp)

add_executable(videoDisplay demo/videoDisplay.cpp)
target_link_libraries(videoDisplay bebop2cpp)

add_executable(fullNavDataOne demo/fullNavData_one.cpp)
target_link_libraries(fullNavDataOne bebop2cpp)

add_executable(fullNavDataTwo demo/fullNavData_two.cpp)
target_link_libraries(fullNavDataTwo bebop2cpp)


add_executable(timestamp demo/timestamp.cpp)
target_link_libraries(timestamp bebop2cpp)

add_executable(test_kotani tuto/test_kotani.cpp)
target_link_libraries(test_kotani bebop2cpp)