#include "NaiveSimulation.h"
#include "PredictiveSimulation.h"
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <omp.h>
#include <memory>
#include<iostream>
#include<cstdlib>

float t=0.0;

static void display(const Simulation& s)
{
	glClearColor((sin(t)+1.0)/2.0,(cos(t)+1.0)/2.0,1.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	Eigen::Vector3f dims(s.boundingplanes[1].offset,s.boundingplanes[3].offset,s.boundingplanes[5].offset);

	glColor3f(1.0f,0.0f,0.0f);
	glBegin(GL_QUADS);
	{
		glVertex3f(-dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],dims[2]);
		glVertex3f(-dims[0],-dims[1],dims[2]);
	}
	glEnd();


	glutSwapBuffers();
}

bool glutloop(const Simulation& s)
{	
	double tstart=omp_get_wtime();
	while((omp_get_wtime()-tstart) < s.dt)
	{
		display(s);
		glutPostRedisplay();
		glutMainLoopEvent();
	}
	t+=omp_get_wtime()-tstart;
	return true;
}

void init_gl(const Simulation& s)
{
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glMatrixMode(GL_PROJECTION);
	Eigen::Vector3f dims(s.boundingplanes[1].offset,s.boundingplanes[3].offset,s.boundingplanes[5].offset);
	//dims/=2.0;
	//double coffset=0.1;
	//double noverf=coffset/(coffset+2.0*s.boundingplanes[5].offset);
/*
	glFrustum(	noverf*s.boundingplanes[0].offset,
			noverf*s.boundingplanes[1].offset,
			noverf*s.boundingplanes[2].offset,
			noverf*s.boundingplanes[3].offset,
			coffset,
			coffset/noverf);*/

	double dwidth=2.0;	//projection at back should be reduced to dwidth percentage of the screen
	Eigen::Matrix4f proj=Eigen::Matrix4f::Zero();
	proj(3,2)=1.0/(2.0*dims[2]);
	proj(3,3)=dwidth-.5;
	proj(0,0)=1.0/dims[0];
	proj(1,1)=1.0/dims[1];
	proj(2,2)=1.0/dims[2];
	//std::cout <<proj << std::endl;
	glLoadMatrixf(proj.data());
	//std::cout << proj*Eigen::Vector4f(-5.0,-3.0,5.0,1.0) << std::endl;;
	//-x -y -z -> -1 -1 -1
	//x -y -z ->
	 

}
void deinit_gl(const Simulation& s)
{

}

int main(int argc,char** argv)
{
	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInit(&argc,argv);

//	glutDisplayFunc(display);
//	glutIdleFunc(idle);
	bool predictive=false;
	double length=30.0;
	if(argc <= 1)
	{
		Simulation::print_help(argv[0]);
		return 0;
	}
	//Scan the arguments for prediction and simulation length
	for(int i=0;i<argc;i++)
	{
		if(argv[i][0]=='-')
		{
			char ch=argv[i][1];
			if(ch=='-')
			{
				ch=argv[i][2];
			}
			//if --predictive option
			if(ch=='p')
			{
				predictive=true;
			}
			if(ch=='l')
			{
				length=atof(argv[++i]);
			}
			if(ch=='h')
			{
				Simulation::print_help(argv[0]);
				return 0;
			}
		}
	}
	std::shared_ptr<Simulation> sim;
	//create a simulation
	if(!predictive)
	{
		sim.reset(new NaiveSimulation(argc,argv));
	}
	else
	{
		sim.reset(new PredictiveSimulation(argc,argv));
	}
	//run the simulation with no visualization callback

	int win=glutCreateWindow("Ball simulation.");

	init_gl(*sim);
	sim->run(length,glutloop);
	glutDestroyWindow(win);
	
	return 0;
}

