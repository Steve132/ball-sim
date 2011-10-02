include_directories(.)

set(SOURCES 
	AxisPlane.cc
	AxisPlane.h
	AxisPlane.inl
	Sphere.cc
	Sphere.inl
	Sphere.h
	Simulation.h
	Simulation.cc
	NaiveSimulation.h
	NaiveSimulation.cc
	PredictiveSimulation.h
	PredictiveSimulation.cc
)

add_library(sphere_sim ${SOURCES})

add_executable(headless_sim headless.cpp)
target_link_libraries(headless_sim sphere_sim)
