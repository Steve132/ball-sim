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
	Barrier.h
	Barrier.cc
)

add_library(sphere_sim ${SOURCES})

add_executable(headless_sim headless.cpp)
target_link_libraries(headless_sim sphere_sim)

find_package(OpenGL)
find_package(GLUT)
if(OPENGL_FOUND AND GLUT_FOUND)
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})

add_executable(glviewer_sim glviewer.cpp)
target_link_libraries(glviewer_sim sphere_sim ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
endif(OPENGL_FOUND AND GLUT_FOUND)
