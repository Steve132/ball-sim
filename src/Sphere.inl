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
	if(ap.offset < 0.0f)
	{
		return position[ap.axis]< (ap.offset+radius);
	}
	else
	{
		return position[ap.axis]> (ap.offset-radius);
	}
}

inline void Sphere::collide(Sphere& c1,const AxisPlane& t)
{
	c1.velocity[t.axis]=-c1.velocity[t.axis]*c1.cor;
	register double r = c1.radius+10e-11; 
	c1.position[t.axis]=t.offset+(t.offset <= 0.0 ? r : -r);
}

inline void Sphere::collide(Sphere& c1, Sphere& c2)
{
    // Physics information on sphere collisions found at:
    // http://www.wheatchex.com/projects/collisions/
    //
    // v1i = the initial velocity of object 1
    // v2i = the initial velocity of object 2
    // m1 = the mass of object 1
    // m2 = the mass of object 2
    // e = the coefficient of restitution (e = 1 for elastic collision)
    // n = normal unit vector drawn from object 1 to object 2
    //
    // c = n . (v1i - v2i) 
    // v1f = v1i - ((m2c)/(m1 + m2))(1 + e)n
    // v2f = v2i + ((m1c)/(m1 + m2))(1 + e)n
    
    Eigen::Vector3d n = c2.position - c1.position;
    Eigen::Vector3d midpoint=(c2.position+c1.position)/2.0;

    n.normalize();
    double c = n.dot((c1.velocity - c2.velocity));
    Eigen::Vector3d c1new = c1.velocity - (((c2.mass * c) / (c1.mass + c2.mass)) * ((1 + c1.cor) * n));
    Eigen::Vector3d c2new = c2.velocity + (((c1.mass * c) / (c1.mass + c2.mass)) * ((1 + c2.cor) * n));
    c1.velocity = c1new;
    c2.velocity = c2new;
    c1.position = midpoint-(c1.radius + 0.01) * n;
    c2.position = midpoint+(c2.radius + 0.01) * n;
    
}

inline void Sphere::update(double dt)
{
	position+=velocity*dt+acceleration*dt*dt*0.5;
	velocity+=acceleration*dt;
}
