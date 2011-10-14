#include "NaiveSimulation.h"

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
NaiveSimulation::NaiveSimulation(int argc,char** argv):
	Simulation(argc,argv)
{
}
