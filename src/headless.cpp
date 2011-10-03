#include "NaiveSimulation.h"
#include "PredictiveSimulation.h"
#include <memory>

bool empty_callback(const Simulation&)
{
	return true;
}

int main(int argc,char** argv)
{
	bool predictive=false;
	double length=30.0;
	
	if(argc <= 1)
	{
		Simulation::print_help(argv[0]);
		return 0;
	}

	for(int i=0;i<argc;i++)
	{
		if(argv[i][0]=='-')
		{
			char ch=argv[i][1];
			if(ch=='-')
			{
				ch=argv[i][2];
			}
			//if --predictive option
			if(ch=='p')
			{
				predictive=true;
			}
			if(ch=='l')
			{
				length=atof(argv[++i]);
			}
			if(ch=='h')
			{
				Simulation::print_help(argv[0]);
				return 0;
			}
		}
	}
	std::shared_ptr<Simulation> sim;

	if(predictive)
	{
		sim.reset(new NaiveSimulation(argc,argv));
	}
	else
	{
		sim.reset(new PredictiveSimulation(argc,argv));
	}
	sim->run(length,empty_callback);
	return 0;
}

