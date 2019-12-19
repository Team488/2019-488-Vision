cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)

project (JohnnyMnemonic)
find_package (OpenCV 4 REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.65.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(shared_mat ../MainReactor/SharedMat.cpp)
add_library(camera ../MainReactor/camera.cpp)

#add_executable (rscolor rscolor.cpp)
add_executable (rsrawcolor rsrawcolor.cpp)
#add_executable (rsir rsir.cpp)
add_executable (rsdepth rsdepth.cpp)
add_executable (rsrawir rsrawir.cpp)
#target_link_libraries(rscolor shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
target_link_libraries(rsrawcolor shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
#target_link_libraries(rsir shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
target_link_libraries(rsdepth shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
target_link_libraries(rsrawir shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)

add_executable (depthPlayback depthPlayback.cpp)
target_link_libraries(depthPlayback ${OpenCV_LIBS})

add_executable (colorPlayback colorPlayback.cpp)
target_link_libraries(colorPlayback ${OpenCV_LIBS})

add_executable (irPlayback irPlayback.cpp)
target_link_libraries(irPlayback ${OpenCV_LIBS})

#add_executable (LocalCamera LocalCam.cpp)
#add_executable (RealSenseCamera RealSenseCam.cpp)
#target_link_libraries(LocalCamera ${OpenCV_LIBS} ${REALSENSE2_LIBRARY})
#target_link_libraries(RealSenseCamera ${OpenCV_LIBS} ${REALSENSE2_LIBRARY})
