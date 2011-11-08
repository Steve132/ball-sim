#ifndef _NAIVE_SIMULATION_H
#define _NAIVE_SIMULATION_H

#include "Simulation.h"
#include<thread>

class NaiveSimulation: public Simulation
{
public:
	NaiveSimulation(int,char**);
	
protected:
	
	void sim_thread(unsigned int thread_id,std::uint64_t timesteps,barrier* b);
	
	virtual void spawn_sim_threads(std::uint64_t timesteps,barrier* b);
	virtual void join_sim_threads();

	std::vector<std::thread> threadpool;
};

#endif
