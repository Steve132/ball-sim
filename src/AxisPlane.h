#ifndef _AXISPLANE_H
#define _AXISPLANE_H

#include<Eigen/Core>

class AxisPlane
{
public:
	AxisPlane(int ax,double d);
	int axis;
	double offset;

	Eigen::Vector3d normal() const;
};

#include"AxisPlane.inl"

#endif
