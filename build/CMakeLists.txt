cmake_minimum_required(VERSION 2.8)
project( Main )
find_package( OpenCV REQUIRED)
add_executable(Main ../main.cpp)
include_directories(../lib)
add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
add_definitions (-D__linux)
target_link_libraries(Main ${OpenCV_LIBS})
set (CMAKE_CXX_FLAGS "-std=gnu++11  -pthread")
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -lrt")