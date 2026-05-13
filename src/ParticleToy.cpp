// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "CircularWireConstraint.h"
#include "Particle.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "imageio.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "MouseSpringForce.h"
#include "Constraint.h"
#include "PointConstraint.h"

#include <GLUT/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle *> pVector, 
	std::vector<Force *> fVector,
	std::vector<Constraint *> cVector, 
	float dt );

/* global variables */

static int N;
static float d;
static int dsim;
static int dump_frames;
static int frame_number;
int solver_type = 0;
float dt = 0.01f;
bool use_sqrt_rodConstraint = false;
static double mouse_ks = 0.50;  // spring stiffness for mouse interaction
static double mouse_kd = 0.10;  // damping

// cloth variables
const int cloth_rows = 5; // rows
const int cloth_columns = 5; // columns
const float cloth_spacing = 0.1f; // spacing between particles in the cloth

// static Particle *pList;
static std::vector<Particle *> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

// static SpringForce *delete_this_dummy_spring = NULL;
// static RodConstraint *delete_this_dummy_rod = NULL;
// static CircularWireConstraint *delete_this_dummy_wire = NULL;
static std::vector<Constraint *> cVector;

static MouseSpringForce* mouseSpring = NULL;
static std::vector <Force*> fVector;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	// clean up particles, forces and constraints
	for (size_t i = 0; i < pVector.size(); i++) {
		delete pVector[i];
	}
	pVector.clear();
	
	for (size_t i = 0; i < fVector.size(); i++) {
		delete fVector[i];
	}
	fVector.clear();
	
	for (size_t i = 0; i < cVector.size(); i++) {
		delete cVector[i];
	}
	cVector.clear();
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

static void init_system(void)
{
  	// clean up from previous runs
  	free_data();

	// particles to be affected by gravity
	std::vector<Particle*> gravityParticles;
	const Vec2f gravityDirection(0.0, -1.0);
	const float gravityStrength = 0.1f;
	const Vec2f center(0.0, 0.0);
	const float dist = 0.5f;

	// create cloth particles in row major order
	for (int i =0; i < cloth_rows; ++i) {
		for (int j = 0; j < cloth_columns; ++j) {
			float x = j * cloth_spacing - (cloth_columns - 1) * cloth_spacing / 2.0f; // center the cloth
			float y = 0.5f - i * cloth_spacing; // start from y=0.5 and go down
			pVector.push_back(new Particle(Vec2f(x, y)));
		}
	}

	// cloth spring connectivity
	for (int i = 0; i < cloth_rows; ++i) {
		for (int j = 0; j < cloth_columns; ++j) {
			int idx = i * cloth_columns + j;
			// structural springs
			// connect to particle on the right
			if (j < cloth_columns - 1) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 1], cloth_spacing, 1.0, 0.5));
			}
			// connect to particle below
			if (i < cloth_rows - 1) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns], cloth_spacing, 1.0, 0.5));
			}
			// shear springs
			// connect to particle diagonally down-right
			if (i < cloth_rows - 1 && j < cloth_columns - 1) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns + 1], cloth_spacing * sqrt(2), 1.0, 0.5));
			}
			// connect to particle diagonally down-left
			if (i < cloth_rows - 1 && j > 0) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns - 1], cloth_spacing * sqrt(2), 1.0, 0.5));
			}
			// flexion springs
			// horizontal (right + 2)
			if (j < cloth_columns - 2) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2], cloth_spacing * 2, 1.0, 0.5));
			}
			// vertical (down + 2)
			if (i < cloth_rows - 2) {
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2 * cloth_columns], cloth_spacing * 2, 1.0, 0.5));
			}
		}
	}

	// add gravity to all particles
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	// fix top row particles
	for (int j = 0; j < cloth_columns; ++j) {
		Particle* p = pVector[j];
		// x dir fix
		cVector.push_back(new PointConstraintX(p, p->m_ConstructPos[0]));
		// y dir fix
		cVector.push_back(new PointConstraintY(p, p->m_ConstructPos[1]));
	}

	// // add circular wire constraint
	// cVector.push_back(new CircularWireConstraint(pVector[12], center, dist));

	// // add rod constraint
	// cVector.push_back(new RodConstraint(pVector[0], pVector[24], dist));
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

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int ii=0; ii< size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces ( void )
{
  for (size_t i = 0; i < fVector.size(); i++) {
    fVector[i]->draw();
  }
}

static void draw_constraints ( void )
{
	for (Constraint* c: cVector) {
		c->draw();
	}
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

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
		pVector[ii]->m_Velocity = Vec2f(0.0, 0.0);
		pVector[ii]->m_Force = Vec2f(0.0, 0.0);
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case '1':
		solver_type = 0;
		printf("Switched to Euler solver.\n");
		break;
	case '2':
		solver_type = 1;
		printf("Switched to Midpoint solver.\n");
		break;
	case '3':
		solver_type = 2;
		printf("Switched to RK4 solver.\n");
		break;
	case 'p':
		dt += 0.001f;
		printf("dt: %f\n", dt);
		break;
	case 'o':
		dt -= 0.001f;
		printf("dt: %f\n", dt);
		break;
	case 'c':
	case 'C':
		clear_data ();
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

	case 's':
		use_sqrt_rodConstraint = !use_sqrt_rodConstraint;
		for (Constraint* c: cVector) {
			RodConstraint* rod = dynamic_cast<RodConstraint*>(c);
			if (rod) {
				rod->m_useSqrt = use_sqrt_rodConstraint;
			}
		}
		printf("Rod constraint now uses %s.\n", use_sqrt_rodConstraint ? "square root" : "squared distance");
		break;

	case ' ':
		dsim = !dsim;
		// clear the sim data if switch from sim to constr mode
		if (!dsim) clear_data();
		break;

	case 'i':
		mouse_ks += 0.05;
		printf("mouse_ks (spring stiffness): %f\n", mouse_ks);
		break;

	case 'u':
		mouse_ks -= 0.05;
		if (mouse_ks < 0) mouse_ks = 0;
		printf("mouse_ks (spring stiffness): %f\n", mouse_ks);
		break;

	case 'k':
		mouse_kd += 0.05;
		printf("mouse_kd (damping): %f\n", mouse_kd);
		break;

	case 'j':
		mouse_kd -= 0.05;
		if (mouse_kd < 0) mouse_kd = 0;
		printf("mouse_kd (damping): %f\n", mouse_kd);
		break;

	case 'h':
		printf("\n=== Keyboard Controls ===\n");
		printf("1/2/3     - Switch solver (Euler/Midpoint/RK4)\n");
		printf("p/o       - Increase/decrease dt (timestep)\n");
		printf("i/u       - Increase/decrease mouse spring stiffness\n");
		printf("k/j       - Increase/decrease mouse damping\n");
		printf("s         - Toggle sqrt formula for rod constraints\n");
		printf("c         - Clear/reset simulation\n");
		printf("d         - Toggle frame dumping\n");
		printf("space     - Toggle simulation/construction mode\n");
		printf("q         - Quit\n");
		printf("Current values:\n");
		printf("  dt: %f\n", dt);
		printf("  mouse_ks: %f\n", mouse_ks);
		printf("  mouse_kd: %f\n", mouse_kd);
		printf("========================\n\n");
		break;
	}
}

// Convert screen coordinates to world coordinates in the range [-1, 1]
Vec2f screenToWorld(int x, int y) {
    float wx = (2.0f * x) / (float)win_x - 1.0f;
    float wy = 1.0f - (2.0f * y) / (float)win_y;
    return Vec2f(wx, wy);
}

static void mouse_func ( int button, int state, int x, int y )
{
	Vec2f worldPos = screenToWorld(x, y);
	
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        // find nearest particle
        Particle* nearest = NULL;
        float minDist = 0.5f; // selection threshold
        for (auto* p : pVector) {
            float d = sqrt(pow(p->m_Position[0]-worldPos[0], 2) + pow(p->m_Position[1]-worldPos[1], 2));
            if (d < minDist) {
                minDist = d;
                nearest = p;
            }
        }
        if (nearest) {
            mouseSpring = new MouseSpringForce(nearest, mouse_ks, mouse_kd);
            mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
            fVector.push_back(mouseSpring);
        }
    } 
    else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        // remove mouse spring
        if (mouseSpring) {
            for (auto it = fVector.begin(); it != fVector.end(); ++it) {
                if (*it == mouseSpring) {
                    fVector.erase(it);
                    break;
                }
            }
            delete mouseSpring;
            mouseSpring = NULL;
        }
    }
	// omx = mx = x;
	// omx = my = y;

	// if(!mouse_down[0]){hmx=x; hmy=y;}
	// if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	// if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	// mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	if (mouseSpring) {
		Vec2f worldPos = screenToWorld(x, y);
		mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
	}
	// mx = x;
	// my = y;
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
	if ( dsim ) simulation_step( pVector, fVector, cVector, dt );
	else        {get_from_UI();remap_GUI();}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();

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
		dt = 0.05f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
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
