#ifndef _NAIVE_SIMULATION_H
#define _NAIVE_SIMULATION_H

#include "Simulation.h"
#include<thread>

class NaiveSimulation: public Simulation
{
public:
	NaiveSimulation(int,char**);
	
	void sim_thread(unsigned int thread_id,unsigned int num_threads,std::uint64_t timesteps);
	
	virtual void spawn_sim_threads(unsigned int num_threads,std::uint64_t timesteps);
	virtual void join_sim_threads();
protected:
	std::vector<std::thread> threadpool;
};

#endif
