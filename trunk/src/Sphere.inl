inline Sphere::Sphere()
{
}

inline bool Sphere::collided(const Sphere& s)
{
	double r=s.radius+radius;
	return 	(s.position-position).norm() < r;
}

inline bool Sphere::collided(const AxisPlane& ap)
{
	double r=position[ap.axis]-ap.offset;
	return r*r < radius*radius;
}

inline void Sphere::collide(Sphere& c1,const AxisPlane& t)
{
	c1.velocity[t.axis]=-c1.velocity[t.axis]*c1.cor;
	register double r = c1.radius+10e-11; 
	c1.position[t.axis]=t.offset+(t.offset <= 0.0 ? r : -r);
}

inline void Sphere::collide(Sphere& c1, Sphere& c2)
{
}

inline void Sphere::update(double dt)
{
	position+=velocity*dt+acceleration*dt*dt*0.5;
	velocity+=acceleration*dt;
}
