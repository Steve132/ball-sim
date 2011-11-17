#ifndef _PREDICTIVE_SIMULATION_H
#define _PREDICTIVE_SIMULATION_H

#include "Simulation.h"
#include<atomic>

class PredictiveSimulation: public Simulation
{
public:
	PredictiveSimulation(int,char**);
	void sim_thread(unsigned int thread_id,std::uint64_t timesteps,barrier* b);

protected:
	class Collision
	{
	public:
		double time;
		bool valid;
		Sphere* sphere1;
		AxisPlane* wall;
		Sphere* sphere2;
		std::atomic<bool> reacted;
		
		Collision();
		Collision(const Collision& c):
			time(c.time),
			valid(c.valid),
			sphere1(c.sphere1),
			wall(c.wall),
			sphere2(c.sphere2),
			reacted((bool)c.reacted)
		{}
		Collision& operator=(const Collision& c)
		{
			time=c.time;
			valid=c.valid;
			sphere1=c.sphere1;
			wall=c.wall;
			sphere2=c.sphere2;
			reacted=(bool)c.reacted;
		}
		void invalidate();
		bool verify();
		void react();
		bool operator<(const Collision& c);
		
	};
	std::shared_ptr<barrier> subframe_barrier;
	Collision repredict(Sphere* sh,double current_time);

	std::vector<Collision> collisions;
	std::vector<std::thread> threadpool;

	virtual void spawn_sim_threads(std::uint64_t timesteps,barrier* bar);
	virtual void join_sim_threads();
	//virtual void update(double dt);
	//virtual void update_threaded(double dt);
};

#endif
