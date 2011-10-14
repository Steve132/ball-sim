#ifndef _SIMULATION_H
#define _SIMULATION_H

#include "AxisPlane.h"
#include "Sphere.h"
#include <vector>
#include <cstdint>
#include <functional>

/**  This class represents an abstract configuration of a single simulation run.
 *
 */
class Simulation
{
protected:
	//The difference between two timestamps in time.
	std::uint64_t current_timestamp;
	std::uint64_t collisions;
	std::uint64_t wall_collisions;
	std::uint64_t sphere_collisions;
	std::uint64_t checks;

	//all 6 bounding planes in the walls.
	
	//true if the simulation is configured to run in a thread
	bool threaded;

	//These are implemented by the particular kind of simulation
	virtual void update(double dt)=0;
	virtual void update_threaded(double dt)=0;

	//The list of all the dynamic spheres
	Sphere* dynamic_spheres;
	unsigned int num_spheres;

	//Creates one sphere randomly in the scene with some default physics
	void initialize_sphere(Sphere& s);
public:
	//Construct a sphere on the command line
	Simulation(int argc,char**);
	virtual ~Simulation();

	static void print_help(const char*);

	//Run a simulation with an optional callback on every loop iteration.
	void run(double seconds,const std::function<bool (const Simulation&)>& callback);

	double dt;
	std::vector<AxisPlane> boundingplanes;
};

#endif
