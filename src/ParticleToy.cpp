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
#include "WindForce.h"
#include "CollisionHandler.h"
#include "AngularSpring.h"

#include <GLUT/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>

/* macros */

/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle *> pVector,
							std::vector<Force *> fVector,
							std::vector<Constraint *> cVector,
							float dt);

/* global variables */

static int N;
static float d;
static int dsim;
static int dump_frames;
static int frame_number;
int solver_type = 0;
enum SceneType
{
	SCENE_PENDULUM = 0,
	SCENE_CLOTH = 1,
	SCENE_HAIR = 2
};
static int scene_type = SCENE_CLOTH; // default to cloth
float dt = 0.01f;
bool use_sqrt_rodConstraint = true;
static double mouse_ks = 0.50;		   // spring stiffness for mouse interaction
static double mouse_kd = 0.10;		   // damping
static WindForce *windForce = NULL;	   // global pointer to the wind force
static Vec2f windDirection(-1.0, 0.0); // blow left
static float windStrength = 0.15f;
static bool enableWind = false; // toggle wind force
static int scene_type = 0; // 0 for cloth, 1 for hair

// cloth variables
int cloth_rows = 5; // rows
int cloth_columns = 5; // columns
float cloth_spacing = 0.045f; // spacing between particles in the cloth
bool fixRowBool = true; // whether to fix in space the top row of the cloth
bool fixCornersBool = false; // whether to fix in space the corners of the cloth
double spring_ks = 10.0f; // spring stiffness for cloth springs
double spring_kd = 3.5f; // damping for cloth springs
float wall_restitution = 0.5f; // restitution coefficient for wall collisions
float wall_friction_coeff = 0.1f; // friction coefficient for wall collisions
float particle_diameter = 0.045f; // diameter of each particle

// hair variables
int hair_segments = 5; // number of segments in the hair
float hair_segment_length = 0.05f; // length of each hair segment

// static Particle *pList;
static std::vector<Particle *> pVector;
static std::vector<Wall> wallVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static std::vector<Constraint *> cVector;

static MouseSpringForce *mouseSpring = NULL;
static std::vector<Force *> fVector;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void)
{
	// clean up particles, forces and constraints
	for (size_t i = 0; i < pVector.size(); i++)
	{
		delete pVector[i];
	}
	pVector.clear();

	for (size_t i = 0; i < fVector.size(); i++)
	{
		delete fVector[i];
	}
	fVector.clear();
	windForce = NULL;  // was owned by fVector, now deleted

	for (size_t i = 0; i < cVector.size(); i++)
	{
		delete cVector[i];
	}
	cVector.clear();
}

static void clear_data(void)
{
	int ii, size = pVector.size();

	for (ii = 0; ii < size; ii++)
	{
		pVector[ii]->reset();
	}
}

/* Scenes */
static void cloth_scene() {
	// clean up from previous runs
  	free_data();

	// initialize walls
	wallVector.clear();
	wallVector.emplace_back(Vec2f(-1.0f, -0.92f), Vec2f(1.0f, -0.92f));
	wallVector.emplace_back(Vec2f(-0.92f, -1.0f), Vec2f(-0.92f, 1.0f));

	// gravity parameters
	const Vec2f gravityDirection(0.0, -1.0);
	const float gravityStrength = 0.1f;

	// create cloth particles in row major order
	for (int i = 0; i < cloth_rows; ++i)
	{
		for (int j = 0; j < cloth_columns; ++j)
		{
			float x = j * cloth_spacing - (cloth_columns - 1) * cloth_spacing / 2.0f; // center the cloth
			float y = 0.9f - i * cloth_spacing;										  // start from almost top and go down
			pVector.push_back(new Particle(Vec2f(x, y)));
		}
	}

	// orient wall normals consistently toward the cloth
	Vec2f clothCenter(0.0f, 0.0f);
	for (Particle* p : pVector) {
		clothCenter += p->m_Position;
	}
	clothCenter /= float(pVector.size());
	for (Wall& wall : wallVector) {
		wall.orientNormalToPoint(clothCenter);
	}

	// cloth spring connectivity
	for (int i = 0; i < cloth_rows; ++i)
	{
		for (int j = 0; j < cloth_columns; ++j)
		{
			int idx = i * cloth_columns + j;
			// structural springs
			// connect to particle on the right
			if (j < cloth_columns - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 1], cloth_spacing, spring_ks, spring_kd));
				// add structural spring to rod constraint
				// cVector.push_back(new RodConstraint(pVector[idx], pVector[idx + 1], cloth_spacing, use_sqrt_rodConstraint));
			}
			// connect to particle below
			if (i < cloth_rows - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns], cloth_spacing, spring_ks, spring_kd));
				// add this as well
				// cVector.push_back(new RodConstraint(pVector[idx], pVector[idx + cloth_columns], cloth_spacing, use_sqrt_rodConstraint));
			}
			// shear springs
			// connect to particle diagonally down-right
			if (i < cloth_rows - 1 && j < cloth_columns - 1)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns + 1], cloth_spacing * sqrt(2), spring_ks, spring_kd));
			}
			// connect to particle diagonally down-left
			if (i < cloth_rows - 1 && j > 0)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + cloth_columns - 1], cloth_spacing * sqrt(2), spring_ks, spring_kd));
			}
			// flexion springs
			// horizontal (right + 2)
			if (j < cloth_columns - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2], cloth_spacing * 2, spring_ks, spring_kd));
			}
			// vertical (down + 2)
			if (i < cloth_rows - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2 * cloth_columns], cloth_spacing * 2, spring_ks, spring_kd));
			}
			// horizontal (right + 2) and vertical (down + 2)
			if (i < cloth_rows - 2 && j < cloth_columns - 2)
			{
				fVector.push_back(new SpringForce(pVector[idx], pVector[idx + 2 * cloth_columns + 2], cloth_spacing * sqrt(8), spring_ks, spring_kd));
			}
		}
	}

	// add gravity to all particles
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));

	if (fixRowBool) {
		// fix top row particles
		for (int j = 0; j < cloth_columns; ++j) {
			Particle* p = pVector[j];
			// x dir fix
			cVector.push_back(new PointConstraintX(p, p->m_ConstructPos[0]));
			// y dir fix
			cVector.push_back(new PointConstraintY(p, p->m_ConstructPos[1]));
			p->m_Pinned = true;
		}
	} else if (fixCornersBool) {
		// fix top row corners
		cVector.push_back(new PointConstraintX(pVector[0], pVector[0]->m_ConstructPos[0]));
		cVector.push_back(new PointConstraintY(pVector[0], pVector[0]->m_ConstructPos[1]));
		cVector.push_back(new PointConstraintX(pVector[cloth_columns - 1], pVector[cloth_columns - 1]->m_ConstructPos[0]));
		cVector.push_back(new PointConstraintY(pVector[cloth_columns - 1], pVector[cloth_columns - 1]->m_ConstructPos[1]));
		pVector[0]->m_Pinned = true;
		pVector[cloth_columns - 1]->m_Pinned = true;
	}

	// add wind force
	windForce = new WindForce(pVector, windDirection, windStrength, enableWind);
	fVector.push_back(windForce);
}

static void hair_scene() {
	free_data();
    float rest_len = hair_segment_length;
    float perturb_offset = rest_len * 0.5f;

    // generate main axis particles and perturbed midpoint particles
    std::vector<Particle*> main_nodes;
    std::vector<Particle*> offset_nodes;

    for (int i = 0; i < hair_segments; ++i) {
        float y = 0.9f - i * rest_len;
        Particle* p = new Particle(Vec2f(0.0f, y));
        pVector.push_back(p);
        main_nodes.push_back(p);

        if (i < hair_segments - 1) {
            // create perturbed particle at the midpoint, offset along X
            Particle* p_off = new Particle(Vec2f(perturb_offset, y - rest_len * 0.5f));
            pVector.push_back(p_off);
            offset_nodes.push_back(p_off);
        }
    }

    // connect structural and triangle-forming springs
    for (int i = 0; i < hair_segments - 1; ++i) {
        // structural edge
        fVector.push_back(new SpringForce(main_nodes[i], main_nodes[i+1], rest_len, spring_ks, spring_kd));
        
        // triangle edges (main node to offset node)
        float diag_len = sqrt(pow(perturb_offset, 2) + pow(rest_len * 0.5f, 2));
        fVector.push_back(new SpringForce(main_nodes[i], offset_nodes[i], diag_len, spring_ks, spring_kd));
        fVector.push_back(new SpringForce(main_nodes[i+1], offset_nodes[i], diag_len, spring_ks, spring_kd));
    }

    // angular torsion springs across the triangles
    for (int i = 0; i < hair_segments - 2; ++i) {
        double angular_ks = spring_ks * 0.005; 
        double angular_kd = spring_kd * 0.005;
        // enforce 180 degrees between consecutive main segments using the triangles
        fVector.push_back(new AngularSpringForce(main_nodes[i], main_nodes[i+1], main_nodes[i+2], M_PI, angular_ks, angular_kd));
    }

    // fix root
    cVector.push_back(new PointConstraintX(main_nodes[0], main_nodes[0]->m_ConstructPos[0]));
    cVector.push_back(new PointConstraintY(main_nodes[0], main_nodes[0]->m_ConstructPos[1]));

    fVector.push_back(new GravityForce(pVector, Vec2f(0.0, -1.0) * 0.1f));
}

static void pendulum_scene()
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.3);
	const Vec2f offset(dist, 0.0);

	// 3 particles in a line
	pVector.push_back(new Particle(center + offset));
	pVector.push_back(new Particle(center + offset + offset));
	pVector.push_back(new Particle(center + offset + offset + offset));

	// gravity and spring
	fVector.push_back(new GravityForce(pVector, gravityDirection * gravityStrength));
	fVector.push_back(new SpringForce(pVector[0], pVector[1], dist, spring_ks, spring_kd));

	// circular wire constraint to first particle
	cVector.push_back(new CircularWireConstraint(pVector[0], center, dist));
	// rod between 1st and 2nd; 2nd and 3rd
	cVector.push_back(new RodConstraint(pVector[1], pVector[2], dist, use_sqrt_rodConstraint));
}

static void init_system(void)
{
	free_data();
	switch (scene_type)
	{
		case 0: cloth_scene(); break;
		case 1: hair_scene(); break;
		case 2: pendulum_scene(); break;
	}
}

static void init_hair()
{
	const int N_HAIR = 10;
	const double segment = 0.08;
	const Vec2f anchor(0.0, 0.7); // top of screen

	// Vertical chain of particles
	for (int i = 0; i < N_HAIR; i++)
	{
		pVector.push_back(new Particle(Vec2f(anchor[0], anchor[1] - i * segment)));
	}

	// Gravity
	fVector.push_back(new GravityForce(pVector, Vec2f(0.0, -0.1)));

	// Structural springs holding the chain together
	for (int i = 0; i < N_HAIR - 1; i++)
	{
		fVector.push_back(new SpringForce(pVector[i], pVector[i + 1], segment, 20.0, 1.0));
	}

	// Angular springs on every triplet (rest angle = pi means "straight")
	for (int i = 0; i < N_HAIR - 2; i++)
	{
		fVector.push_back(new AngularSpring(pVector[i], pVector[i + 1], pVector[i + 2], M_PI, 5.0, 0.1));
	}

	// Pin the top particle — mark Pinned so implicit Euler enforces Δv=0
	pVector[0]->m_Pinned = true;
	cVector.push_back(new PointConstraintX(pVector[0], anchor[0]));
	cVector.push_back(new PointConstraintY(pVector[0], anchor[1]));
}

static void init_system(void)
{
	free_data();
	switch (scene_type)
	{
	case SCENE_PENDULUM:
		init_pendulum();
		break;
	case SCENE_CLOTH:
		init_cloth();
		break;
	case SCENE_HAIR:
		init_hair();
		break;
	}
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	// Write frames if necessary.
	if (dump_frames)
	{
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0)
		{
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char *buffer = (unsigned char *)malloc(w * h * 4 * sizeof(unsigned char));
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

	glutSwapBuffers();
}

static void draw_particles(void)
{
	int size = pVector.size();

	for (int ii = 0; ii < size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces(void)
{
	for (size_t i = 0; i < fVector.size(); i++)
	{
		fVector[i]->draw();
	}
}

static void draw_constraints(void)
{
	for (Constraint *c : cVector)
	{
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0] && !mouse_shiftclick[0] && !mouse_shiftclick[2])
		return;

	i = (int)((mx / (float)win_x) * N);
	j = (int)(((win_y - my) / (float)win_y) * N);

	if (i < 1 || i > N || j < 1 || j > N)
		return;

	if (mouse_down[0])
	{
	}

	if (mouse_down[2])
	{
	}

	hi = (int)((hmx / (float)win_x) * N);
	hj = (int)(((win_y - hmy) / (float)win_y) * N);

	if (mouse_release[0])
	{
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for (ii = 0; ii < size; ii++)
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

static void key_func(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '1':
		if (scene_type == SCENE_HAIR) {
			printf("Euler is unstable for hair angular springs — keeping Implicit Euler.\n");
			break;
		}
		solver_type = 0;
		printf("Switched to Euler solver.\n");
		break;
	case '2':
		if (scene_type == SCENE_HAIR) {
			printf("Midpoint is unstable for hair angular springs — keeping Implicit Euler.\n");
			break;
		}
		solver_type = 1;
		printf("Switched to Midpoint solver.\n");
		break;
	case '3':
		solver_type = 2;
		printf("Switched to RK4 solver.\n");
		break;
	case '4':
		solver_type = 3;
		printf("Switched to Implicit Euler solver.\n");
		break;
	case '5':
		solver_type = 4;
		printf("Switched to Verlet solver.\n");
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
		clear_data();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data();
		exit(0);
		break;

	case 's':
		scene_type = (scene_type + 1) % 3; // toggle between scenes
		init_system();
		printf("Switched to %s scene.\n", scene_type == 0 ? "cloth" : (scene_type == 1 ? "hair" : "pendulum"));
		break;

	case 'r':
		use_sqrt_rodConstraint = !use_sqrt_rodConstraint;
		for (Constraint *c : cVector)
		{
			RodConstraint *rod = dynamic_cast<RodConstraint *>(c);
			if (rod)
			{
				rod->m_useSqrt = use_sqrt_rodConstraint;
			}
		}
		printf("Rod constraint now uses %s.\n", use_sqrt_rodConstraint ? "square root" : "squared distance");
		break;

	case ' ':
		dsim = !dsim;
		// clear the sim data if switch from sim to constr mode
		if (!dsim)
			clear_data();
		break;

	case 'i':
		spring_ks += 0.5;
		printf("spring_ks (spring stiffness): %f\n", spring_ks);
		break;

	case 'u':
		spring_ks -= 0.5;
		if (spring_ks < 0)
			spring_ks = 0;
		printf("spring_ks (spring damping): %f\n", spring_ks);
		break;

	case 'k':
		spring_kd += 0.5;
		printf("spring_kd (damping): %f\n", spring_kd);
		break;

	case 'j':
		spring_kd -= 0.5;
		if (spring_kd < 0)
			spring_kd = 0;
		printf("spring_kd (damping): %f\n", spring_kd);
		break;

	case 'w':
		enableWind = !enableWind;
		if (windForce)
		{
			windForce->setEnabled(enableWind);
		}
		if (enableWind && solver_type == 0) {
			solver_type = 2;
			printf("Wind enabled. Auto-switched to RK4 for stability.\n");
		} else {
			printf("Wind %s\n", enableWind ? "enabled" : "disabled");
		}
		break;

	case 'f':
		fixRowBool = !fixRowBool;
		printf("Row fixing %s\n", fixRowBool ? "enabled" : "disabled");
		init_system(); // rebuild scene to apply changes
		break;

	case 'v':
	{
		int choice;
		printf("\n=== Modify Simulation Parameters ===\n");
		printf("1. Cloth Rows (current: %d)\n", cloth_rows);
		printf("2. Cloth Columns (current: %d)\n", cloth_columns);
		printf("3. Cloth Spacing (current: %f)\n", cloth_spacing);
		printf("4. Spring Stiffness [ks] (current: %f)\n", spring_ks);
		printf("5. Spring Damping [kd] (current: %f)\n", spring_kd);
		printf("6. Time Step [dt] (current: %f)\n", dt);
		printf("7. Hair Segments (current: %d)\n", hair_segments);
		printf("8. Hair Segment Length (current: %f)\n", hair_segment_length);
		printf("Select an option (1-8): ");
		
		std::cin >> choice;

		if (choice == 1) {
			printf("Enter new number of cloth rows: ");
			std::cin >> cloth_rows;
			if (cloth_rows < 2) cloth_rows = 2; // prevent zero/negative size crashes
			init_system(); // rebuilds scene 
			printf("Scene rebuilt with %d rows.\n", cloth_rows);
		} 
		else if (choice == 2) {
			printf("Enter new number of cloth columns: ");
			std::cin >> cloth_columns;
			if (cloth_columns < 2) cloth_columns = 2;
			init_system(); // rebuilds scene
			printf("Scene rebuilt with %d columns.\n", cloth_columns);
		} 
		else if (choice == 3) {
			printf("Enter new cloth spacing: ");
			std::cin >> cloth_spacing;
			if (cloth_spacing <= 0) cloth_spacing = 0.01f; // prevent non-positive spacing
			init_system(); // rebuilds scene
			printf("Scene rebuilt with cloth spacing = %f.\n", cloth_spacing);
		}
		else if (choice == 4) {
			printf("Enter new spring stiffness (ks): ");
			std::cin >> spring_ks;
			init_system(); // rebuilds scene
			printf("Scene rebuilt with ks = %f.\n", spring_ks);
		} 
		else if (choice == 5) {
			printf("Enter new spring damping (kd): ");
			std::cin >> spring_kd;
			init_system(); // rebuilds scene
			printf("Scene rebuilt with kd = %f.\n", spring_kd);
		} 
		else if (choice == 6) {
			printf("Enter new time step (dt): ");
			std::cin >> dt;
			printf("Time step updated to dt = %f.\n", dt);
		}
		else if (choice == 7) {
			printf("Enter new hair segments amount.\n");
			std::cin >> hair_segments;
			if (hair_segments < 2) hair_segments = 2;
			init_system();
			printf("Scene rebuilt with hair segments = %d.\n", hair_segments);
		}
		else if (choice == 8) {
			printf("Enter new hair segment length.\n");
			std::cin >> hair_segment_length;
			if (hair_segment_length <= 0) hair_segment_length = 0.01f;
			init_system();
			printf("Scene rebuilt with hair segment length = %f.\n", hair_segment_length);
		}
		else {
			printf("Invalid selection.\n");
		}
		break;
	}

	case 'h':
		printf("\n=== Keyboard Controls ===\n");
		printf("v         - Open parameter modification menu in console\n");
		printf("1/2/3/4/5 - Switch solver (Euler/Midpoint/RK4/ImplicitEuler/Verlet)\n");
		printf("p/o       - Increase/decrease dt (timestep)\n");
		printf("i/u       - Increase/decrease spring stiffness\n");
		printf("k/j       - Increase/decrease spring damping\n");
		printf("w         - Toggle wind force\n");
		printf("s         - Switch between cloth and hair scenes\n");
		printf("r         - Toggle sqrt formula for rod constraints\n");
		printf("c         - Clear/reset simulation\n");
		printf("d         - Toggle frame dumping\n");
		printf("space     - Toggle simulation/construction mode\n");
		printf("q         - Quit\n");
		
		printf("Current values:\n");
		printf("  dt: %f\n", dt);
		printf("  spring_ks: %f\n", spring_ks);
		printf("  spring_kd: %f\n", spring_kd);
		printf("========================\n\n");
		break;
	}
}

// Convert screen coordinates to world coordinates in the range [-1, 1]
Vec2f screenToWorld(int x, int y)
{
	float wx = (2.0f * x) / (float)win_x - 1.0f;
	float wy = 1.0f - (2.0f * y) / (float)win_y;
	return Vec2f(wx, wy);
}

static void mouse_func(int button, int state, int x, int y)
{
	Vec2f worldPos = screenToWorld(x, y);

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		// find nearest particle
		Particle *nearest = NULL;
		float minDist = 0.5f; // selection threshold
		for (auto *p : pVector)
		{
			float d = sqrt(pow(p->m_Position[0] - worldPos[0], 2) + pow(p->m_Position[1] - worldPos[1], 2));
			if (d < minDist)
			{
				minDist = d;
				nearest = p;
			}
		}
		if (nearest)
		{
			mouseSpring = new MouseSpringForce(nearest, mouse_ks, mouse_kd);
			mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
			fVector.push_back(mouseSpring);
		}
	}
	else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		// remove mouse spring
		if (mouseSpring)
		{
			for (auto it = fVector.begin(); it != fVector.end(); ++it)
			{
				if (*it == mouseSpring)
				{
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

static void motion_func(int x, int y)
{
	if (mouseSpring)
	{
		Vec2f worldPos = screenToWorld(x, y);
		mouseSpring->updateMousePosition(worldPos[0], worldPos[1]);
	}
	// mx = x;
	// my = y;
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	if ( dsim ) {
		simulation_step( pVector, fVector, cVector, dt );
		CollisionHandler::handleWallCollisions(pVector, wallVector, wall_restitution, wall_friction_coeff);
		CollisionHandler::handleParticleCollisions(pVector, particle_diameter, wall_restitution);

		if (scene_type == 1) {
			const float hair_drag = 5.0f;
			for (auto* p : pVector) {
				if (!p->m_Pinned)
					p->m_Velocity *= (1.0f - hair_drag * dt);
			}
		}

		bool explosion_detected = false;
		for (auto* p : pVector) {
			if (!std::isfinite(p->m_Position[0]) || !std::isfinite(p->m_Position[1]) ||
			    !std::isfinite(p->m_Velocity[0]) || !std::isfinite(p->m_Velocity[1])) {
				explosion_detected = true;
				break;
			}
			float dx = p->m_Position[0] - p->m_ConstructPos[0];
			float dy = p->m_Position[1] - p->m_ConstructPos[1];
			if (dx*dx + dy*dy > 25.0f) { 
				explosion_detected = true;
				break;
			}
		}
		if (explosion_detected) {
			printf("Explosion detected — resetting scene.\n");
			clear_data();
		}
	} else {
		get_from_UI();
		remap_GUI();
	}

	glutSetWindow(win_id);
	glutPostRedisplay();
}

static void display_func(void)
{
	pre_display();

	draw_forces();
	draw_constraints();
	draw_particles();
	CollisionHandler::drawWalls(wallVector);

	post_display();
}

/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Particletoys!");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	if (argc == 1)
	{
		N = 64;
		dt = 0.05f;
		d = 5.f;
		fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
				N, dt, d);
	}
	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf("\n\nHow to use this application:\n\n");
	printf("\t Toggle construction/simulation display with the spacebar key\n");
	printf("\t Dump frames by pressing the 'd' key\n");
	printf("\t Quit by pressing the 'q' key\n");

	printf("\n=== Keyboard Controls ===\n");
	printf("v         - Open parameter modification menu in console\n");
	printf("1/2/3     - Switch solver (Euler/Midpoint/RK4)\n");
	printf("p/o       - Increase/decrease dt (timestep)\n");
	printf("i/u       - Increase/decrease mouse spring stiffness\n");
	printf("k/j       - Increase/decrease mouse damping\n");
	printf("s         - Toggle sqrt formula for rod constraints\n");
	printf("w         - Toggle wind force\n");
	printf("f         - Toggle fixing top row of cloth\n");
	printf("c         - Clear/reset simulation\n");
	printf("d         - Toggle frame dumping\n");
	printf("space     - Toggle simulation/construction mode\n");
	printf("q         - Quit\n");
	printf("Current values:\n");
	printf("  dt: %f\n", dt);
	printf("  spring_ks: %f\n", spring_ks);
	printf("  spring_kd: %f\n", spring_kd);
	printf("========================\n\n");

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	init_system();

	win_x = 512;
	win_y = 512;
	open_glut_window();

	glutMainLoop();

	exit(0);
}
