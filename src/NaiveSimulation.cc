#include "NaiveSimulation.h"
#include<functional>

void NaiveSimulation::sim_thread(unsigned int thread_id,unsigned int num_threads,std::uint64_t timesteps)
{
	Simulation::Statistics& cstats=stats[thread_id];
	for(cstats.current_timestamp=0;(cstats.current_timestamp<timesteps) && running;)
	{
		for(unsigned int i = thread_id; i < num_spheres; i+=num_threads)
		{
			// Now that all spheres have been updated, we are interested in checking
			// sphere-sphere collisions, as well as sphere-wall collisions.
			// First, let's loop through all walls, checking each sphere as we do so.
			for(int j = 0; j < boundingplanes.size(); j++)
			{
				// at each sphere, check to see if it has collided with a wall
				if(dynamic_spheres[i].collided(boundingplanes[j]))
				{
					// if so, increment collided counter and calculate
					// collision effects
					cstats.collisions++;
					cstats.wall_collisions++;
					Sphere::collide(dynamic_spheres[i], boundingplanes[j]);
				}
				// increment check counter
				cstats.checks++;
			}
			// Second, let's check each sphere with each other sphere.  As we progress
			// through the list, each successive iteration will only need to begin one
			// sphere further down the list, otherwise we would be checking every
			// sphere twice.
			for(int j = (i + 1); j < num_spheres; j++)
			{
				// check to see if spheres [i] and [j] have collided
				if(dynamic_spheres[i].collided(dynamic_spheres[j]))
				{
					// if so, increment collided counter and calculate
					// collision effects
					cstats.collisions++;
					cstats.sphere_collisions++;
					Sphere::collide(dynamic_spheres[i], dynamic_spheres[j]);
				}
				//increment check counter
				cstats.checks++;
			}
			
			dynamic_spheres[i].update(dt);
		}

		cstats.current_timestamp++;
		wait_stats(cstats.current_timestamp);
	}
}

void NaiveSimulation::join_sim_threads()
{
	for(unsigned int i=0;i<threadpool.size();i++)
	{
		threadpool[i].join();
	}
}

void NaiveSimulation::spawn_sim_threads(unsigned int num_threads,std::uint64_t timesteps)
{
	for(unsigned int i=0;i<num_threads;i++)
	{
		threadpool.push_back(std::thread(std::bind(&NaiveSimulation::sim_thread,this,i,num_threads,timesteps)));
	}
}

/*
void NaiveSimulation::update(double dt)
{
	// First, we must update the positions of all spheres in the scene
	for(int i = 0; i < num_spheres; i++)
	{
		dynamic_spheres[i].update(dt);
	}

	// Now that all spheres have been updated, we are interested in checking
	// sphere-sphere collisions, as well as sphere-wall collisions.
	// First, let's loop through all walls, checking each sphere as we do so.
	for(int i = 0; i < num_spheres; i++)
	{
		for(int j = 0; j < boundingplanes.size(); j++)
		{
			// at each sphere, check to see if it has collided with a wall
			if(dynamic_spheres[i].collided(boundingplanes[j]))
			{
				// if so, increment collided counter and calculate
				// collision effects
				collisions++;
				wall_collisions++;
				Sphere::collide(dynamic_spheres[i], boundingplanes[j]);
			}
			// increment check counter
			checks++;
		}
	}

	// Second, let's check each sphere with each other sphere.  As we progress
	// through the list, each successive iteration will only need to begin one
	// sphere further down the list, otherwise we would be checking every
	// sphere twice.
	for(int i = 0; i < num_spheres; i++)
	{
		for(int j = (i + 1); j < num_spheres; j++)
		{
			// check to see if spheres [i] and [j] have collided
			if(dynamic_spheres[i].collided(dynamic_spheres[j]))
			{
				// if so, increment collided counter and calculate
				// collision effects
				collisions++;
				sphere_collisions++;
				Sphere::collide(dynamic_spheres[i], dynamic_spheres[j]);
			}
			//increment check counter
			checks++;
		}
	}
}
void NaiveSimulation::update_threaded(double dt)
{
	// First, we must update the positions of all spheres in the scene
	int i;
	#pragma omp parallel for
	for(i = 0; i < num_spheres; i++)
	{
		dynamic_spheres[i].update(dt);
	}

	// Now that all spheres have been updated, we are interested in checking
	// sphere-sphere collisions, as well as sphere-wall collisions.
	// First, let's loop through all walls, checking each sphere as we do so.
	#pragma omp parallel for
	for(i = 0; i < num_spheres; i++)
	{
		for(int j = 0; j < boundingplanes.size(); j++)
		{
			// at each sphere, check to see if it has collided with a wall
			if(dynamic_spheres[i].collided(boundingplanes[j]))
			{
				// if so, increment collided counter and calculate
				// collision effects
				collisions++;
				wall_collisions++;
				Sphere::collide(dynamic_spheres[i], boundingplanes[j]);
			}
			// increment check counter
			checks++;
		}
	}

	// Second, let's check each sphere with each other sphere.  As we progress
	// through the list, each successive iteration will only need to begin one
	// sphere further down the list, otherwise we would be checking every
	// sphere twice.
	#pragma omp parallel for
	for(i = 0; i < num_spheres; i++)
	{
		for(int j = (i + 1); j < num_spheres; j++)
		{
			// check to see if spheres [i] and [j] have collided
			if(dynamic_spheres[i].collided(dynamic_spheres[j]))
			{
				// if so, increment collided counter and calculate
				// collision effects
				collisions++;
				sphere_collisions++;
				Sphere::collide(dynamic_spheres[i], dynamic_spheres[j]);
			}
			//increment check counter
			checks++;
		}
	}
}
*/
NaiveSimulation::NaiveSimulation(int argc,char** argv):
	Simulation(argc,argv)
{
}
