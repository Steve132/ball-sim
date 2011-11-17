#include "PredictiveSimulation.h"
#include<functional>
using namespace Eigen;


PredictiveSimulation::Collision::Collision()
{
	invalidate();
	reacted.exchange(false);
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
		// check to see if this sphere is already colliding with another sphere
		if((sh->collided(dynamic_spheres[i])) && (sh!=&dynamic_spheres[i]))
		{
			//uh-oh, we are!  quick, cause another collision
			pr.collided = true;
			pr.timeoffset = 0.0;
			c.time=pr.timeoffset+current_time;
			c.valid=true;
			c.sphere1=sh;
			c.sphere2=&dynamic_spheres[i];
			c.wall=NULL;
			break;
		}
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
		// check to see if this sphere is already colliding with a wall
		if(sh->collided(boundingplanes[i]))
		{
			//uh-oh, we are!  quick, cause another collision
			pr.collided = true;
			pr.timeoffset = 0.0;
			c.time=pr.timeoffset+current_time;
			c.valid=true;
			c.sphere1=sh;
			c.sphere2=NULL;
			c.wall=&boundingplanes[i];
			break;
		}
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
			current_collision.reacted.exchange(false); //this is important
		}
		
		subframe_barrier->wait();
		//subframe barrier here.  Now spheres won't possibly change in the middle of reaction or prediction.
		
		//Invariants, no invalids should reach this point.
		for(unsigned int i=thread_id;i<num_spheres;i+=num_threads)
		{
			Collision& current_collision=collisions[i];//there should be a cross-pair check..along with thread protection.
			//if pending
			// we're touching a sphere and doing calculations, count this
			// as a check
			cstats.checks++;
			if(current_collision.valid && (current_collision.time <= current_time)) //&& current_collision.verify())
			{
				//if it happened, react
				if(current_collision.verify())
				{
					// we've verified a valid collision, update statistics
					cstats.collisions++;
					//if it happened, if this is a sphere-sphere collision
					if(current_collision.wall)
					{
						// this is a wall collision
						cstats.wall_collisions++;
						current_collision.react();
						current_collision.reacted=true;
					}
					else
					{
						// this is a sphere collision
						cstats.sphere_collisions++;
						Collision& other_collision=collisions[current_collision.sphere2-&dynamic_spheres[0]];
						if(	!(other_collision.valid &&
							other_collision.time <= current_time &&
							other_collision.sphere2==&dynamic_spheres[i] &&
								other_collision.reacted) &&
							!current_collision.reacted.exchange(true))
						{
							current_collision.react();//react happens once exactly.  Short circuits
						}
					}
				}
				else
				{
					// verify failed
				}
				//if it didn't happen or it did, the collision is no longer valid
				current_collision.invalidate();
			}
		}
		
		//Optional subframe barrier?  No, as spheres can update before or after the react phase that modifies them
		//subframe_barrier->wait();
		
		for(unsigned int i=thread_id;i<num_spheres;i+=num_threads)
		{
			dynamic_spheres[i].update(dt);
		}
		if(cstats.current_timestamp == 34)
		{
			// breakpoint here
			int donkey = 10;
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
