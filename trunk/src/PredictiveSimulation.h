#ifndef _PREDICTIVE_SIMULATION_H
#define _PREDICTIVE_SIMULATION_H

#include "Simulation.h"

class PredictiveSimulation: public Simulation
{
public:
	PredictiveSimulation(int,char**);
	virtual void update(double dt);
	virtual void update_threaded(double dt);
};

#endif
