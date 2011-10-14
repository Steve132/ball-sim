#include "NaiveSimulation.h"
#include "PredictiveSimulation.h"
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <omp.h>
#include <memory>
#include<iostream>
#include<cstdlib>

Sphere sph;

static void display(const Simulation& s)
{
	glClearColor(0.0,0.0,0.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	Eigen::Vector3f dims(s.boundingplanes[1].offset,s.boundingplanes[3].offset,s.boundingplanes[5].offset);


	glDisable(GL_LIGHTING);

	//do walls
	glBegin(GL_QUADS);
	{
		//back wall
		glColor3f(.75f,.75f,.8f);
		glVertex3f(-dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],dims[2]);
		glVertex3f(-dims[0],-dims[1],dims[2]);

		//top wall
		glColor3f(.75f,.8f,.75f);
		glVertex3f(-dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],dims[1],-dims[2]);
		glVertex3f(-dims[0],dims[1],-dims[2]);

		//bottom wall
		glColor3f(.8f,.75f,.8f);
		glVertex3f(-dims[0],-dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],-dims[2]);
		glVertex3f(-dims[0],-dims[1],-dims[2]);

		//right wall
		glColor3f(.8f,.75f,.75f);
		glVertex3f(dims[0],dims[1],-dims[2]);
		glVertex3f(dims[0],dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],dims[2]);
		glVertex3f(dims[0],-dims[1],-dims[2]);

		//left wall
		glColor3f(.75f,.8f,.8f);
		glVertex3f(-dims[0],dims[1],-dims[2]);
		glVertex3f(-dims[0],dims[1],dims[2]);
		glVertex3f(-dims[0],-dims[1],dims[2]);
		glVertex3f(-dims[0],-dims[1],-dims[2]);
		
	}
	glEnd();
	/*
	for(unsigned int i=0;i<s.num_spheres;i++)
	{
		glLoadIdentity();
		const Sphere& sph=&s.dynamic_spheres[i];
		glTranslated(sph.
	}*/
	glEnable(GL_LIGHTING);
	glColor3f(1.0,0.0,0.0);
		
	glTranslated(sph.position[0],sph.position[1],sph.position[2]);
	glutSolidSphere(sph.radius,64,32);

	glLoadIdentity();

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
	sph.update(s.dt);
	bool bc=sph.collided(s.boundingplanes[2]);
	std::cout << bc << std::endl;
	if(bc)
	{
		Sphere::collide(sph,s.boundingplanes[2]);
	}
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
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//std::cout << proj*Eigen::Vector4f(-5.0,-3.0,5.0,1.0) << std::endl;;
	//-x -y -z -> -1 -1 -1
	//x -y -z ->

	s.initialize_sphere(sph);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	float lightpos[]={0.0,1.0,0.0,0.0};
	glLightfv(GL_LIGHT0,GL_POSITION,lightpos);

	 

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

