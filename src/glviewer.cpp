#include "NaiveSimulation.h"
#include "PredictiveSimulation.h"
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <omp.h>
#include <memory>
#include<iostream>

static void display()
{
	glClearColor(1.0,0.0,0.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glutSwapBuffers();
}

static void idle()
{
	glutPostRedisplay();
}	

bool glutloop(const Simulation& s)
{	
	double tstart=omp_get_wtime();
	while((omp_get_wtime()-tstart) < s.dt)
	{
		display();
		glutPostRedisplay();
		glutMainLoopEvent();
	}
	return true;
}

void init_gl(const Simulation& s)
{

}
void deinit_gl(const Simulation& s)
{

}

int main(int argc,char** argv)
{
	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInit(&argc,argv);

	glutDisplayFunc(display);
	glutIdleFunc(idle);
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

