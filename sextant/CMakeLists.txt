cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)
set(OPENCV_ENABLE_NONFREE)

project (sextant)
find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable (setup setup.cpp)
target_link_libraries(setup ${OpenCV_LIBS})

add_executable (runtime runtime.cpp)
target_link_libraries(runtime ${OpenCV_LIBS})

add_executable (refVideoWriter refVideoWriter.cpp)
target_link_libraries(refVideoWriter ${OpenCV_LIBS})

add_executable (bestFrame bestFrame.cpp)
target_link_libraries(bestFrame ${OpenCV_LIBS})
