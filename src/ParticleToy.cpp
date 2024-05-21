// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "ParticleSystem.h"
#include "ClothParticleSystem.h"
#include "Particle.h"
#include "Force.h"
#include "imageio.h"
#include "Solver.h"
#include "Constraint.h"
#include "LinearSolver.h"

#include "SampleSystems.h"

#include <iostream>
#include <limits>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

/* macros */

/* global variables */

static int N;
static float d;
static int dsim;
static int dump_frames;
static int frame_number;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static ParticleSystem* particleSystem = nullptr;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	delete particleSystem;
}

static void init_system(void)
{
	// ClothSystem* system = new ClothSystem();
	// system->initSystem(pVector, fVector);
	// system->setCornerConstraints(pVector, cVector);
	// fVector.push_back(new GravityForce(pVector));

	// setIntegrationHook(RungeKutta);

	// delete system;
	// int ii, size = pVector.size();
	
	particleSystem = cloth1();
	particleSystem->clear_data();
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {

	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {

	}

	omx = mx;
	omy = my;
}

// static void remap_GUI()
// {
// 	int ii, size = pVector.size();
// 	for(ii=0; ii<size; ii++)
// 	{
// 		//pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
// 		//pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
// 		//pVector[ii]->clearForce();

// 		pVector[ii]->reset();
// 	}
// }

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		particleSystem->clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if ( dsim ) particleSystem->simulationStep();
	else        {get_from_UI();}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	particleSystem->drawForces();
	particleSystem->drawConstraints();
	particleSystem->drawParticles();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d d=%g\n",
			N, d );
	} else {
		N = atoi(argv[1]);
		d = atof(argv[2]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

