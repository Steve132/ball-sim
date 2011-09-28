#ifndef _SPHERE_H
#define _SPHERE_H

#include<Eigen/Core>
#include "AxisPlane.h"

class Sphere
{
public:
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	double mass;
	double radius;
	double cor;	//coefficient of restitution

	Sphere();

	bool collided(const Sphere& c);
	bool collided(const AxisPlane& c);
	void update(double t);

	static void collide(Sphere& c1,Sphere& c2);
	static void collide(Sphere& c1,const AxisPlane& t);
};

#include "Sphere.inl"
#endif
