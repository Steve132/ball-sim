#ifndef _SIMULATION_H
#define _SIMULATION_H

#include "AxisPlane.h"
#include "Sphere.h"
#include <vector>
#include <cstdint>
#include <functional>
#include <atomic>
#include <iostream>
/**  This class represents an abstract configuration of a single simulation run.
 *
 */
class Simulation
{
public:
	struct barrier
	{
		std::vector<std::atomic_uint_fast32_t> cur_threads;
		std::uint_fast32_t total_waitthreads;
		barrier(const std::uint_fast32_t& twt);
		void wait(const std::uint_fast32_t& twt);
	};
protected:
	//The difference between two timestamps in time.
	struct Statistics
	{
		std::uint64_t current_timestamp;
		std::uint64_t collisions;
		std::uint64_t wall_collisions;
		std::uint64_t sphere_collisions;
		std::uint64_t checks;
		Statistics();
		Statistics& operator+=(const Statistics&);
	};
	std::vector<Statistics> stats;

	//all 6 bounding planes in the walls.
	
	//true if the simulation is configured to run in a thread
	int num_threads;
	//true if the simulation is still running;
	bool running;

	//These are implemented by the particular kind of simulation
	virtual void spawn_sim_threads(std::uint64_t timesteps,Simulation::barrier* bar)=0;
	virtual void join_sim_threads()=0;
	//virtual void run_sim_threaded(double dt)=0;

	//The list of all the dynamic spheres

	
	void wait_stats(std::uint64_t);
	//Creates one sphere randomly in the scene with some default physics
//	void initialize_sphere(Sphere& s);
public:

	//Construct a sphere on the command line
	Simulation(int argc,char**);
	virtual ~Simulation();

	static void print_help(const char*);

	//Run a simulation with an optional callback on every loop iteration.
	void run(double seconds,const std::function<bool (const Simulation&)>& callback);

	double dt;
	std::vector<AxisPlane> boundingplanes;
	Sphere* dynamic_spheres;
	unsigned int num_spheres;
	void initialize_sphere(Sphere& s) const;
};

#endif
