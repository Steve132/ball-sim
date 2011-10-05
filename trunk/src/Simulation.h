#ifndef _SIMULATION_H
#define _SIMULATION_H

#include "AxisPlane.h"
#include "Sphere.h"
#include <vector>
#include <cstdint>
#include <functional>

class Simulation
{
protected:
	double dt;
	std::uint64_t current_timestamp;
	std::uint64_t collisions;
	std::uint64_t wall_collisions;
	std::uint64_t sphere_collisions;
	std::uint64_t checks;
	std::vector<AxisPlane> boundingplanes;
	bool threaded;

	virtual void update(double dt)=0;
	virtual void update_threaded(double dt)=0;

	Sphere* dynamic_spheres;
	unsigned int num_spheres;
	void initialize_sphere(Sphere& s);
public:
	Simulation(int argc,char**);
	virtual ~Simulation();

	static void print_help(const char*);

	void run(double seconds,const std::function<bool (const Simulation&)>& callback);
};

#endif
