cmake_minimum_required(VERSION 3.1)
project(test_deadreckon)

## Setting 
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

## Include
#include_directories(test_deadreckon ../common ../third-party/imgui ../)
#include(${CMAKE_SOURCE_DIR}/CMake/opengl_config.cmake)
#include(/home/donghyun/CLionProjects/untitled/CMake/opengl_config.cmake)

## Target
add_executable(test_deadreckon main.cpp)

## Link libraries
find_package(OpenCV REQUIRED)
set(DEPENDENCIES ${OpenCV_LIBS})
target_link_libraries( test_deadreckon ${OpenCV_LIBS} )

set(BOOST_LIBS date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
target_link_libraries(test_deadreckon ${Boost_LIBRARIES})
find_package(Threads REQUIRED)
target_link_libraries(test_deadreckon ${CMAKE_THREAD_LIBS_INIT})
