#ifndef _NAIVE_SIMULATION_H
#define _NAIVE_SIMULATION_H

#include "Simulation.h"

class NaiveSimulation: public Simulation
{
public:
	NaiveSimulation(int,char**);
	virtual void update(double dt);
	virtual void update_threaded(double dt);
};

#endif
