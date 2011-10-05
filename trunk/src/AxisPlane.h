#ifndef _AXISPLANE_H
#define _AXISPLANE_H

#include<Eigen/Core>

/** This check represents a wall
*
*/
class AxisPlane
{
public:
	AxisPlane(int ax,double d);
	int axis;//the axis of the wall.  The direction the normal points along.

	double offset;//the distance from the wall to the centerplane parallel to it.

	//the normal of the wall.
	Eigen::Vector3d normal() const;
};

#include"AxisPlane.inl"

#endif
