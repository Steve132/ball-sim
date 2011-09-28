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
)

add_library(sphere_sim ${SOURCES})
