cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)

project (videotest)
find_package (OpenCV REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.65.0 REQUIRED COMPONENTS date_time)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(shared_mat SharedMat.cpp)
add_library(camera camera.cpp)

add_executable (videosend videosend.cpp)
target_link_libraries(videosend ${OpenCV_LIBS})
add_executable (videorec videorec.cpp)
target_link_libraries(videorec ${OpenCV_LIBS})

# Shared memory demos
add_executable (shared_reader shared_reader.cpp)
target_link_libraries(shared_reader shared_mat ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

add_executable (shared_source shared_source.cpp)
target_link_libraries(shared_source shared_mat ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

#Camera demos
add_executable (shared_camera_source shared_camera_source.cpp)
target_link_libraries(shared_camera_source shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

add_executable (shared_camera_reader shared_camera_reader.cpp)
target_link_libraries(shared_camera_reader shared_mat camera ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

if (WIN32)
else()
    target_link_libraries(shared_reader -lrt)
    target_link_libraries(shared_source -lrt)
    target_link_libraries(shared_camera_source -lrt)
    target_link_libraries(shared_camera_reader -lrt)
endif()