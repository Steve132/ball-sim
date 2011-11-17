#include "PredictiveSimulation.h"
#include<functional>
using namespace Eigen;


PredictiveSimulation::Collision::Collision()
{
	invalidate();
}
void PredictiveSimulation::Collision::invalidate()
{
	time=10e15;
	valid=false;
	sphere1=NULL;
	wall=NULL;
	sphere2=NULL;
}
bool PredictiveSimulation::Collision::verify()
{
	if(wall != NULL)
	{
		return sphere1->collided(*wall);
	}
	else
	{
		return sphere1->collided(*sphere2);
	}
}
void PredictiveSimulation::Collision::react()
{
	if(wall != NULL)
	{
		Sphere::collide(*sphere1,*wall);
	}
	else
	{
		Sphere::collide(*sphere1,*sphere2);
	}
}

//PredictionResult predict(const Sphere& a,const Sphere& b);
//PredictionResult predict(const Sphere& a,const AxisPlane& axis);

PredictiveSimulation::Collision PredictiveSimulation::repredict(Sphere* sh,double current_time)
{
	Collision c;
	c.time=current_time;
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=10e15;

	for(unsigned int i=0;i<num_spheres;i++)
	{
		PredictionResult prt=predict(*sh,dynamic_spheres[i]);
		if(prt.collided && (prt.timeoffset < pr.timeoffset) && sh!=&dynamic_spheres[i])
		{
			pr=prt;
			c.time=pr.timeoffset+current_time;
			c.valid=true;
			c.sphere1=sh;
			c.sphere2=&dynamic_spheres[i];
			c.wall=NULL;
		}
	}
	for(unsigned int i=0;i<boundingplanes.size();i++)
	{
		PredictionResult prt=predict(*sh,boundingplanes[i]);
		//std::cout << (prt.collided ? "Valid" : "Invalid") << "Collision detected at time " << c.time << " with wall " << i << std::endl;

		if(prt.collided && (prt.timeoffset < pr.timeoffset))
		{
			pr=prt;
			c.time=pr.timeoffset+current_time;
			c.valid=true;
			c.sphere1=sh;
			c.sphere2=NULL;
			c.wall=&boundingplanes[i];
		}
	}
	if(!c.valid)
	{
		//throw std::runtime_error("No predicted intersection found!  Not possible!");
	}
	return c;
}

void PredictiveSimulation::sim_thread(unsigned int thread_id,std::uint64_t timesteps,barrier* bar)
{
	Simulation::Statistics& cstats=stats[thread_id];

	for(cstats.current_timestamp=0;(cstats.current_timestamp<timesteps) && running;)
	{
		double current_time=cstats.current_timestamp*dt;
		//std::cout << current_time << std::endl;
		//Each object updates

		//Each invalid collision is re-predicted
		for(unsigned int i=thread_id;i<num_spheres;i+=num_threads)
		{
			Collision& current_collision=collisions[i];
			if(!current_collision.valid)
			{
				current_collision=repredict(&dynamic_spheres[i],current_time);
				/*std::cout << "Prediction made to occur at " << current_collision.time << " between " << i << " and ";
				if(current_collision.wall)
					std::cout << "Wall " << (int)(current_collision.wall-&boundingplanes[0]) << std::endl;
				else
					std::cout << (int)(current_collision.sphere2-dynamic_spheres) << std::endl;*/
			}
		}
		
		subframe_barrier->wait();
		//subframe barrier here.  Now spheres won't possibly change in the middle of reaction or prediction.
		
		//Invariants, no invalids should reach this point.
		for(unsigned int i=thread_id;i<num_spheres;i+=num_threads)
		{
			Collision& current_collision=collisions[i];//there should be a cross-pair check..along with thread protection.
			//atomic boolean to see if reacted this frame.
			//if pending
			if(current_collision.valid && current_collision.time <= current_time)
			{
				//if it happened, react
				if(current_collision.verify())
				{
					current_collision.react();
				}
				//if it didn't happen or it did, the collision is no longer valid
				current_collision.invalidate();
			}
		}
		
		subframe_barrier->wait();
		
		for(unsigned int i=thread_id;i<num_spheres;i+=num_threads)
		{
			dynamic_spheres[i].update(dt);
		}
		
		cstats.current_timestamp++;
		//wait_stats(cstats.current_timestamp);
		bar->wait();
	}

}

void PredictiveSimulation::spawn_sim_threads(std::uint64_t timesteps,barrier* bar)
{
	subframe_barrier.reset(new barrier(num_threads));
	collisions.resize(num_spheres);
	for(int i=0;i<num_threads;i++)
	{
		threadpool.push_back(std::thread(std::bind(&PredictiveSimulation::sim_thread,this,i,timesteps,bar)));
	}
}
void PredictiveSimulation::join_sim_threads()
{
	for(int i=0;i<threadpool.size();i++)
	{
		threadpool[i].join();
	}
}

PredictiveSimulation::PredictiveSimulation(int argc,char** argv):
	Simulation(argc,argv)
{
}
