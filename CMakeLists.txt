cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenGL REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})
find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
#target_link_libraries( ${PROJECT_NAME} realsense2 ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${realsense2_LIBRARY})

set(BOOST_LIBS date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/common
${PROJECT_SOURCE_DIR}/third-party/imgui
${PROJECT_SOURCE_DIR}/sensors
${PROJECT_SOURCE_DIR}/dead_reckoning
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc

include/Frame.h
include/KeyFrame.h
include/Tracking.h
include/LocalMapping.h

src/IMU/configparam.h
src/IMU/configparam.cpp

src/IMU/imudata.h
src/IMU/imudata.cpp
src/IMU/IMUPreintegrator.h
src/IMU/IMUPreintegrator.cpp
src/IMU/so3.cpp
src/IMU/so3.h
src/IMU/NavState.h
src/IMU/NavState.cpp

src/IMU/g2otypes.h
src/IMU/g2otypes.cpp

sensors/robot_control.h
sensors/robot_control.cpp
sensors/gyro.h
sensors/gyro.cpp
sensors/TimeoutSerial.h
sensors/TimeoutSerial.cpp

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
cholmod
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
realsense2 
${OPENGL_LIBRARIES} 
${realsense2_LIBRARY}
${Boost_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
)

## Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#add_executable(rgbd_tum_VI
#Examples/RGB-D/rgbd_tum_VI.cc)
#target_link_libraries(rgbd_tum_VI ${PROJECT_NAME})

add_executable(rgbd_VI_rt
Examples/RGB-D/rgbd_VI_rt.cc)
target_link_libraries(rgbd_VI_rt ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})

#add_executable(stereo_euroc_VI
#Examples/Stereo/stereo_euroc_VI.cc)
#target_link_libraries(stereo_euroc_VI ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

#add_executable(mono_euroc_VI
#Examples/Monocular/mono_euroc_VI.cc)
#target_link_libraries(mono_euroc_VI ${PROJECT_NAME})

#add_executable(mono_realsense_VI
#Examples/Monocular/mono_realsense_VI.cc)
#target_link_libraries(mono_realsense_VI ${PROJECT_NAME})


## Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
#add_executable(bin_vocabulary
#Vocabulary/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})
