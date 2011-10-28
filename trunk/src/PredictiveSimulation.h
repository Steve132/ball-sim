#ifndef _PREDICTIVE_SIMULATION_H
#define _PREDICTIVE_SIMULATION_H

#include "Simulation.h"

class PredictiveSimulation: public Simulation
{
public:
	PredictiveSimulation(int,char**);
	virtual void spawn_sim_threads(unsigned int num_threads,std::uint64_t timesteps);
	virtual void join_sim_threads();
	//virtual void update(double dt);
	//virtual void update_threaded(double dt);
};

#endif
