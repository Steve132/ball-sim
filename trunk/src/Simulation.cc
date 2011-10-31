#include "Simulation.h"
#include <mmintrin.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>
#include <thread>
#include <omp.h>

Simulation::Statistics::Statistics():
	current_timestamp(0),
	collisions(0),
	wall_collisions(0),
	sphere_collisions(0),
	checks(0)
{}

Simulation::Statistics& Simulation::Statistics::operator+=(const Statistics& st)
{
	current_timestamp=st.current_timestamp;
	collisions+=st.collisions;
	wall_collisions+=st.wall_collisions;
	sphere_collisions+=st.sphere_collisions;
	checks+=st.checks;
	return *this;
}
void Simulation::print_help(const char* arg0)
{
	std::cout << "Usage: " << arg0 << "[options]" << std::endl;
	std::cout << "\t-x <float> -y <float> -z <float>\tSets the dimensions of the room of interest.  Defaults to {5,3,5}" << std::endl;
	std::cout << "\t-n <int> | --num_spheres <int>\t Sets the number of spheres to generate.  Defaults to 100" << std::endl;
	std::cout << "\t-t <int>| --threaded\t Enables threaded mode with some threads.  Disabled by default, if no arg is given then the number of threads is the maximum " << std::endl;
	std::cout << "\t-h | --help\t Prints this help message." << std::endl;
	std::cout << "\t-f <float> | --fps <float>\tSets the simulation timedelta as frames per second. Defaults to 64.0" << std::endl;
	std::cout << "\t-l <float> | --length <float>\tSets the length of the simulation to run in seconds.  Defaults to 30.0" << std::endl;
	std::cout << "\t-p | --predictive\t Enables predictive mode.  Disabled by default" << std::endl;

}
Simulation::Simulation(int argc,char** argv)
{
//Make defaults for the simulation
	Eigen::Vector3d dimensions(5.0,3.0,5.0);
	num_threads=1;
	unsigned int seedhash=0;
	num_spheres=100;
	dt=1.0/64.0;

	//Scan the arguments
	for(int i=0;i<argc;i++)
	{
		if(argv[i][0]=='-')
		{
			char c=argv[i][1];
			if(c=='-')
			{
				c=argv[i][2];
			}
			switch(tolower(c))
			{
			case 'x':
				{
					dimensions[0]=atof(argv[++i])/2.0;
					break;
				}
			case 'y':
				{
					dimensions[1]=atof(argv[++i])/2.0;
					break;
				}
			case 'z':
				{
					dimensions[2]=atof(argv[++i])/2.0;
					break;
				}
			case 'n':
				{
					num_spheres=atoi(argv[++i]);
					break;
				}
			case 't':
				{
					if(argv[i+1][0]=='-')
					{
						num_threads=1;
					}
					else
					{
						num_threads=omp_get_max_threads();
					}
					break;
				}
			case 'f':
				{
					dt=1.0/atof(argv[++i]);
					break;
				}
			};
		}
	}
/*
	const Eigen::Vector3d& dimensions,double timedelta,unsigned int seedhash,unsigned int ballcount,bool t):
	dt(timedelta),
	current_timestamp(0),
	threaded(t),
	num_spheres(ballcount)
{*/
	//Make the spheres using aligned memory.
	dynamic_spheres=reinterpret_cast<Sphere*>(_mm_malloc(num_spheres*sizeof(Sphere),16));

	for(int i=0;i<3;i++)
	{
		boundingplanes.push_back(AxisPlane(i,-dimensions[i]));
		boundingplanes.push_back(AxisPlane(i,dimensions[i]));
	}
	//use the seed value to initialize the simulation the same every run.
	srand(seedhash);
	for(Sphere* s=dynamic_spheres;s != (dynamic_spheres+num_spheres);++s)
	{
		new (s) Sphere;
		initialize_sphere(*s);
	}
}
Simulation::~Simulation()
{
	_mm_free(dynamic_spheres);
}

static double randfloat(double lower,double upper)
{
	double u=(double)rand()/(double)RAND_MAX;
	return lower+(upper-lower)*u;
}

void Simulation::initialize_sphere(Sphere& s) const
{	
	//find the minimum offset size...
	//this doesn't really work, it makes all room sizes equal.  Instead, you should choose a size, say, 
	//.5 meters max, and 5 centimeters
	/*double minbounds=10e20;
	for(int i=0;i<3;i++)
	{
		if(boundingplanes[2*i+1].offset < minbounds)
		{
			minbounds=boundingplanes[2*i+1].offset;
		}
	}*/

	//make a ball between 5 cm and 50 cm
	s.radius=randfloat(0.05,0.5);
	//put the ball in the room somewhere.
	s.position[0]=randfloat(boundingplanes[0].offset+s.radius,boundingplanes[1].offset-s.radius);
	s.position[1]=randfloat(boundingplanes[2].offset+s.radius,boundingplanes[3].offset-s.radius);
	s.position[2]=randfloat(boundingplanes[4].offset+s.radius,boundingplanes[5].offset-s.radius);

	//average density of a rubber-like ball
	double density=1200.0+randfloat(-100.0,100.0); //kg/m^3
	density/=1000.0;
	s.mass = s.radius * s.radius * s.radius * 4.0 / 3.0 * M_PI * density;

	//average cor (bounciness) is bouncy.
	s.cor=randfloat(.9,.96);
	//apply gravity and shoot it in a random direction.
	s.acceleration=Eigen::Vector3d(0.0,-9.8,0.0);
	s.velocity[0]=randfloat(-20.0,20.0);
	s.velocity[1]=randfloat(-20.0,20.0);
	s.velocity[2]=randfloat(-20.0,20.0);
	
}

void Simulation::wait_stats(std::uint64_t ts0)
{
	bool waiting=false;
	while(waiting)
	{
		waiting=false;
		for(int i=0;i<stats.size();i++)
		{
			if(stats[i].current_timestamp != ts0)
			{
				waiting=true;
				break;
			}
		}
	}
}

void Simulation::run(double seconds,const std::function<bool (const Simulation&)>& callback)
{
	std::uint64_t current_timestamp=0;
	//Calculate the real-world time
	double tinit=omp_get_wtime();
	//Calculate the number of timesteps
	std::uint64_t timesteps=seconds/dt;

	//While the callback doesn't trigger a quit, keep going.
	running=true;
	//unsigned int num_threads;
	
	/*if(threaded)
		num_threads=omp_get_max_threads();
	else
		num_threads=1;*/
	//if(num_threads > 1)
	//	num_threads-=1;
	
	stats.resize(num_threads);
	
	barrier b(num_threads+1);
	
	spawn_sim_threads(timesteps,b);
	
	for(current_timestamp=0;(current_timestamp < timesteps) && (running);current_timestamp++)
	{
		wait_stats(current_timestamp+1);
		running=callback(*this);
		//b.wait();
	}
	
	join_sim_threads();
	
	double tend=omp_get_wtime();
	
	Statistics gstats;
	
	for(int i=0;i<num_threads;i++)
	{
		gstats+=stats[i];
	}

	std::cout << "Run statistics:" << std::endl;
	std::cout << "total_timesteps\t" << gstats.current_timestamp << std::endl;
	std::cout << "total_real_world_time\t"      << tend-tinit << std::endl;	

	std::cout << "total_collisions\t" << gstats.collisions << std::endl;
	std::cout << "wall_collisions\t" << gstats.wall_collisions << std::endl;
	std::cout << "sphere_collisions\t" << gstats.sphere_collisions << std::endl;
	std::cout << "total_checks\t" << gstats.checks <<std::endl;
	std::cout << "average_checks/timestep\t" << double(gstats.checks)/double(timesteps) << std::endl;

	std::cout << "average_timesteps/second\t" << double(gstats.current_timestamp)/(tend-tinit)<<std::endl;
	std::cout << "average_milliseconds/timestep\t" << (tend-tinit)/double(gstats.current_timestamp/1000.0) << std::endl;
	std::cout << "time_dilation\t" << (gstats.current_timestamp*dt)/(tend-tinit) << std::endl;
	
	std::cout << "total_threads\t" << num_threads << std::endl;
}

