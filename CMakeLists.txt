cmake_minimum_required(VERSION 2.8)

project(sphere_sim)

find_package(OpenMP)
if(MSVC)
	set(FLAGS_BASE "")
else(MSVC)
	set(FLAGS_BASE "-std=c++0x ")
endif(MSVC)

if(OPENMP_FOUND)
	set(FLAGS_BASE "${FLAGS_BASE} ${OpenMP_CXX_FLAGS}")
else(OPENMP_FOUND)
	message(ERROR "OpenMP support was not found, but is required for threading and timing")
endif(OPENMP_FOUND)

set(CMAKE_CXX_FLAGS ${FLAGS_BASE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_subdirectory(src)
