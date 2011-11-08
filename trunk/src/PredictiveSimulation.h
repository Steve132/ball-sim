#ifndef _PREDICTIVE_SIMULATION_H
#define _PREDICTIVE_SIMULATION_H

#include "Simulation.h"

class PredictiveSimulation: public Simulation
{
public:
	PredictiveSimulation(int,char**);
protected:
	virtual void spawn_sim_threads(std::uint64_t timesteps,Simulation::barrier* bar);
	virtual void join_sim_threads();
	//virtual void update(double dt);
	//virtual void update_threaded(double dt);
};

#endif
