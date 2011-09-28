AxisPlane::AxisPlane(int ax,double d):
	axis(ax),
	offset(d)
{}

Eigen::Vector3d AxisPlane::normal() const
{
	Eigen::Vector3d v(0.0,0.0,0.0);
	v[axis]=-offset;
	v.normalize();
	return v;
}

