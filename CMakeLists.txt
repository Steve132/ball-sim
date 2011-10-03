cmake_minimum_required(VERSION 2.8)

project(sphere_sim)

set(FLAGS_BASE "-std=c++0x -fopenmp")
set(CMAKE_CXX_FLAGS ${FLAGS_BASE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_subdirectory(src)
