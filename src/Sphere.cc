#include "Sphere.h"
#include<Eigen/Eigenvalues>
using namespace Eigen;

#define  EPS 10e-7
/*
static PredictionResult polymin4(double* coeffs)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;

	std::complex<double> eig[4];
	
	//double alpha,beta,gamma;
	double a=coeffs[4],b=coeffs[3],c=coeffs[2],d=coeffs[1],e=coeffs[0];
	double ba=b/a,ca=c/a,da=d/a,ea=e/a;
	
	std::complex<double> alpha((3.0/8.0)*ba*ba+ca),
	beta((1.0/8.0)*ba*ba*ba-(ba*ca)/2.0+da),
	gamma((-3.0/256.0)*ba*ba*ba*ba+(1.0/16.0)*ca*ba*ba-(1.0/4.0)*ba*da+ea);
	
	if(abs(beta) < EPS)
	{
		std::complex<double> sd1,sd2,sdi;
		sdi=sqrt(alpha*alpha-4*gamma);
		sd1=sqrt((-alpha+sdi)/2.0);
		sd2=sqrt((-alpha-sdi)/2.0);
		roots[0]=-sd1-ba/4.0;
		roots[1]= sd1-ba/4.0;
		roots[2]=-sd2-ba/4.0;
		roots[3]= sd2-ba/4.0;
	}
	else
	{
		std::complex<double> P(-alpha*alpha/12.0-gamma);
		std::complex<double> Q(-alpha*alpha*alpha/108.0+alpha*gamma/3.0-beta*beta/8.0);
		std::complex<double> R=-Q/2.0+sqrt(Q*Q/4.0+P*P*P/27.0);
		
	}
	
	double tvalue=-1.0;
	for(int i=0;i<4;i++) //5 maybe?
	{
		std::complex<double> ev=eig[i];
		if(ev.real() > 0.0 && (std::abs(ev.imag()) < EPS))
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
}*/

static PredictionResult polymin1(double* coeffs)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;
	
	//0=c0+c1*t
	//-c0=c1t
	//-c0/c1t
	if(abs(coeffs[1]) < EPS)
	{
		return pr;
	}
	double tv=-coeffs[0]/coeffs[1];
	if(tv > 0.0)
	{
		pr.timeoffset=tv;
		pr.collided=true;
	}
	return pr;
}

static PredictionResult polymin2(double* coeffs)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;
	
	//std::complex<double> root[2];
	if(std::abs(coeffs[2]) < EPS)
	{
		return polymin1(coeffs);
	}
	//std::complex<double> a(coeffs[2],0.0),b(coeffs[1],0.0),c(coeffs[0],0.0);
	double a=coeffs[2],b=coeffs[1],c=coeffs[0];
	
	double disc=b*b-a*c*4.0;
	if(disc < 0.0)
	{
		return pr;
	}
	
	double sdc=sqrt(disc);
	double r1=(-b+sdc)/(a*2.0);
	double r2=(-b-sdc)/(a*2.0);
	
	double tvalue=-1.0;
	if(r1 > 0.0)
	{
		tvalue=r1;
	}
	if(r2 > 0.0 && ((r2 < tvalue) || (tvalue < 0.0)))
	{
		tvalue=r2;
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
	//The accelerations are shared so ad is ALWAYS 0.  Therefore this is a simple quadratic.
	//return polymin4(coeffs);
	return polymin2(coeffs);
}

PredictionResult predict(const Sphere& a,const AxisPlane& axis)
{
	PredictionResult pr;
	pr.collided=false;
	pr.timeoffset=0.0;
	
	Eigen::Vector3d n=axis.normal();
	double sd=(a.position-axis.offset*Eigen::Vector3d::Ones()).dot(n)-a.radius;
	double vd=a.velocity.dot(n);
	double ad=a.acceleration.dot(n);
	
	double coeffs[3]={
		sd,
		vd,
		ad/2.0
	};
		
	return polymin2(coeffs);
}
