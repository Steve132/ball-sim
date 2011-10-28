#ifndef _SPHERE_H
#define _SPHERE_H

#include<Eigen/Core>
#include "AxisPlane.h"

/** A sphere object
*
*/
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

	//collided is called to check intersections between this sphere and the argument
	bool collided(const Sphere& c);
	bool collided(const AxisPlane& c);

	//Update the ball according to physical properties of the ball
	void update(double t);

	//React both arguments to a collision between the objects
	static void collide(Sphere& c1,Sphere& c2);
	static void collide(Sphere& c1,const AxisPlane& t);
};

#include "Sphere.inl"
#endif
