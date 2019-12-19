cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)

project (morten)
find_package (OpenCV 4 REQUIRED)
FIND_PACKAGE(Threads REQUIRED)
find_package(realsense2 REQUIRED)

set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.65.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(shared_mat ../MainReactor/SharedMat.cpp)
add_library(camera ../MainReactor/camera.cpp)
add_library(smootour smootour.cpp)

#add_executable (vectorvision vectorvision.cpp)
#target_link_libraries(vectorvision smootour ${OpenCV_LIBS})

#add_executable (morten morten.cpp)
#target_link_libraries(morten smootour ${OpenCV_LIBS})

#add_executable (mortenintel mortenintel.cpp)
#target_link_libraries(mortenintel smootour ${OpenCV_LIBS} realsense2::realsense2)

add_executable (realsense realsense.cpp)
target_link_libraries(realsense smootour ${OpenCV_LIBS} realsense2::realsense2)

add_executable (morten mortenshared.cpp)
target_link_libraries(morten smootour shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} -lrt)
