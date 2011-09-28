#ifndef _SIMULATION_H
#define _SIMULATION_H

#include "AxisPlane.h"
#include "Sphere.h"
#include<vector>
#include<cstdint>
#include<functional>

class Simulation
{
protected:
	double dt;
	std::uint64_t current_timestamp;
	std::vector<AxisPlane> boundingplanes;
	bool threaded;

	virtual void update(double dt)=0;

	Sphere* dynamic_spheres;
	unsigned int num_spheres;
	void initialize_sphere(Sphere& s);
public:
	Simulation(const Eigen::Vector3d& dimensions,double timedelta,unsigned int seedhash,unsigned int ballcount,bool threaded);
	~Simulation();

	void run(double seconds,const std::function<bool (const Simulation&)>& callback);
};

#endif
