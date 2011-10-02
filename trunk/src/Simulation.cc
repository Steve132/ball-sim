#include "Simulation.h"
#include <mmintrin.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <cstring>

Simulation::Simulation(int argc,char** argv):
	current_timestamp(0)
{
	Eigen::Vector3d dimensions(5.0,3.0,5.0);
	threaded=false;
	unsigned int seedhash=0;
	num_spheres=100;
	dt=1.0/64.0;

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
					dimensions[0]=atof(argv[++i]);
					break;
				}
			case 'y':
				{
					dimensions[1]=atof(argv[++i]);
					break;
				}
			case 'z':
				{
					dimensions[2]=atof(argv[++i]);
					break;
				}
			case 'n':
				{
					num_spheres=atoi(argv[++i]);
					break;
				}
			case 't':
				{
					threaded=true;
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
	dynamic_spheres=reinterpret_cast<Sphere*>(_mm_malloc(num_spheres*sizeof(Sphere),16));

	for(int i=0;i<3;i++)
	{
		boundingplanes.push_back(AxisPlane(i,-dimensions[i]));
		boundingplanes.push_back(AxisPlane(i,dimensions[i]));
	}
	
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
	return (double)rand()/(double)RAND_MAX;
}

void Simulation::initialize_sphere(Sphere& s)
{	
	double minbounds=10e20;
	for(int i=0;i<3;i++)
	{
		if(boundingplanes[2*i+1].offset < minbounds)
		{
			minbounds=boundingplanes[2*i+1].offset;
		}
	}

	s.radius=randfloat(minbounds/30.0,minbounds/10.0);
	s.position.x()=randfloat(boundingplanes[0].offset+s.radius,boundingplanes[1].offset-s.radius);
	s.position.y()=randfloat(boundingplanes[2].offset+s.radius,boundingplanes[3].offset-s.radius);
	s.position.z()=randfloat(boundingplanes[4].offset+s.radius,boundingplanes[5].offset-s.radius);

	double density=1200.0+randfloat(-100.0,100.0); //kg/m^3
	density/=1000.0;
	s.mass = s.radius * s.radius * s.radius * 4.0 / 3.0 * M_PI * density;
	s.cor=randfloat(.8,.9999);
	s.acceleration=Eigen::Vector3d(0.0,-9.8,0.0);
	s.velocity.x()=randfloat(-20.0,20.0);
	s.velocity.y()=randfloat(-20.0,20.0);
	s.velocity.z()=randfloat(-20.0,20.0);
	
}

void Simulation::run(double seconds,const std::function<bool (const Simulation&)>& callback)
{
	std::uint64_t timesteps=seconds/dt;
	bool quit=false;
	if(threaded)
	{
		for(current_timestamp=0;current_timestamp < timesteps && (!quit);current_timestamp++)
		{
			update(dt);
			quit=callback(*this);
		}
	}
	else
	{
		for(current_timestamp=0;current_timestamp < timesteps && (!quit);current_timestamp++)
		{
			update_threaded(dt);
			quit=callback(*this);
		}	
	}
}

