cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)
SET(json_spirit_DIR ../json)

project (targetingcomputer)
find_package (OpenCV 4 REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.65.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${json_spirit_DIR}/include)
link_directories(${json_spirit_DIR}/lib)

add_library(shared_mat ../MainReactor/SharedMat.cpp)
add_library(camera ../MainReactor/camera.cpp)


#add_executable (targetingcomputer targetingcomputer.cpp)
#target_link_libraries(targetingcomputer czmq zmq ${OpenCV_LIBS})

#add_executable (rstc rstc.cpp)
#target_link_libraries(rstc czmq zmq ${OpenCV_LIBS} ${REALSENSE2_LIBRARY})

add_executable (targetingcomputer rstc.cpp)
target_link_libraries(targetingcomputer czmq zmq shared_mat camera json_spirit ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
