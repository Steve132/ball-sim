#include "Sphere.h"
#include<Eigen/Eigenvalues>
using namespace Eigen;

static PredictionResult polymin(double* coeffs)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;
	
	//form the monic polynomial companion matrix
	//this would be more complex under 0g, which a corner case oddly, but it is fine in this case
	
	Eigen::Matrix4d companion=Eigen::Matrix4d::Zero(4,4);
	for(int i=0;i<4;i++)
	{
		companion(i,3)=-coeffs[i]/coeffs[4];
	}
	for(int i=1;i<4;i++)
	{
		companion(i,i-1)=1.0;
	}
	
	Eigen::Vector4cd eig=companion.eigenvalues();
	
	//Perform roots here.  Use eigenvalues of companion matrix to find all roots (some or all will be complex)
	//If all roots are complex or negative, return false
	//otherwise, return the smallest positive root.
	
	double tvalue=-1.0;
	for(int i=0;i<4;i++) //5 maybe?
	{
		std::complex<double> ev=eig[i];
		if(ev.real() > 0.0 && (std::abs(ev.imag()) < 0.00001))
		{	
			if(tvalue < 0.0)
				tvalue=10e11; //to initialize value
			if(ev.real() < tvalue)
				tvalue=ev.real();
		}
	}
	if(tvalue < 0.0)
	{
		pr.collided=false;
	}
	else
	{
		pr.collided=true;
		pr.timeoffset=tvalue;
	}
	return pr;
}
PredictionResult predict(const Sphere& s1,const Sphere& s2)
{
	//find roots of 4d polynomial for collision
	//||s1(t)-s2(t)||==r1+r2
	Eigen::Vector3d ad=s2.acceleration-s1.acceleration;
	Eigen::Vector3d vd=s2.velocity-s1.velocity;
	Eigen::Vector3d sd=s2.position-s1.position;
	double distance=s2.radius+s1.radius;
	
	//||sd+vd*t+.5*t*t*ad||^2==distance^2
	//||sd+vd*t+.5*t*t*ad||^2-distance^2==0
	//==x(t)^2+y(t)^2+z(t)^2-distance^2
	
	//x(t)
	//==(sd.x+vd.x*t+.5*t*t*ad.x)^2==convolution...
	//==(sd.x+vd.x*t+.5*t*t*ad.x)*sd.x+(sd.x+vd.x*t+.5*t*t*ad.x)*vd.x*t+(sd.x+vd.x*t+.5*t*t*ad.x)*.5*t*t*ad.x
	
	//==sd.x^2
	//+(vd.x*sd.x+sd.x*vd.x)*t
	//+(.5*ad.x*sd.x+vd.x^2*+sd.x*.5*ad.x)*tt
	//+(.5*ad.x*vd.x+vd.x*ad.x*.5)*ttt
	//+(.5*.5*ad.x*ad.x)tttt
	
	//==sd.x^2
	//+(2.0*vd.x*sd.x)*t
	//+(ad.x*sd.x+vd.x^2)*tt
	//+(ad.x*vd.x)*ttt
	//+(.25 ad.x^2)*tttt
	
	double coeffs[5]={
		sd.dot(sd)-distance*distance,
		sd.dot(vd)*2.0,
		sd.dot(ad)+vd.dot(vd),
		ad.dot(vd),
		ad.dot(ad)*.25
	};
	return polymin(coeffs);
}

PredictionResult predict(const Sphere& a,const AxisPlane& axis)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;
	
	Eigen::Vector3d n=axis.normal();
	double sd=a.position.dot(n)-axis.offset;
	double vd=a.velocity.dot(n);
	double ad=a.acceleration.dot(n);
	double distance=a.radius;
	
	double coeffs[5]={
		sd*sd-distance*distance,
		sd*vd*2.0,
		sd*ad+vd*vd,
		ad*vd,
		ad*ad*.25
	};
	return polymin(coeffs);
}
