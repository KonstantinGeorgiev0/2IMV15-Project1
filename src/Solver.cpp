#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Force.h"
#include "Constraint.h"
#include "ConstraintSolver.h"


#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)

extern int solver_type;
extern float dt;

/* get state from particle into dst */
void ParticleGetState(std::vector<Particle*> pVector, std::vector<float> &dst) {
	// clear dst
	dst.clear();
	
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++)
	{
		dst.push_back(pVector[ii]->m_Position[0]);
		dst.push_back(pVector[ii]->m_Position[1]);
		dst.push_back(pVector[ii]->m_Velocity[0]);
		dst.push_back(pVector[ii]->m_Velocity[1]);
	}
}

/* set state from src into particle */
void ParticleSetState(std::vector<Particle*> pVector, const std::vector<float> &src) {
	int ii, size = pVector.size();
	int ind = 0;

	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = src[ind++];
		pVector[ii]->m_Position[1] = src[ind++];
		pVector[ii]->m_Velocity[0] = src[ind++];
		pVector[ii]->m_Velocity[1] = src[ind++];
	}
}

/* calculate derivative, place in dst */
void ParticleDerivative(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	std::vector<float> &dst)
{
	int ii, size = pVector.size();
	dst.clear();
	
	// zero the force accumulation
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Force = Vec2f(0.0, 0.0);
	}

	// compute forces
	for (auto* f : fVector) {
		f->apply();
	}

	// apply constraints
	solve_constraints(pVector, cVector);

	// fill dst with derivatives
	for(ii=0; ii<size; ii++)
	{
		if (pVector[ii]->m_Pinned) {
			// pinned particles don't move
			dst.push_back(0.0f); // xdot = 0
			dst.push_back(0.0f); // ydot = 0
			dst.push_back(0.0f); // vdot_x = 0
			dst.push_back(0.0f); // vdot_y = 0
		} else {
			// xdot = v
			dst.push_back(pVector[ii]->m_Velocity[0]);
			dst.push_back(pVector[ii]->m_Velocity[1]);
			// vdot = f/m
			dst.push_back(pVector[ii]->m_Force[0] / pVector[ii]->m_Mass);
			dst.push_back(pVector[ii]->m_Force[1] / pVector[ii]->m_Mass);
		}
	}
	
}

// Euler solver
void euler_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, derivative;
	// get state
	ParticleGetState(pVector, state);
	// get derivative 
	ParticleDerivative(pVector, fVector, cVector, derivative);

	// Euler method: x_new = x + deltaT * xdot
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += derivative[ii] * deltaT;
	}

	ParticleSetState(pVector, state);
}

// Midpoint solver
void midpoint_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, derivative, midState, midDerivative;
	// get state
	ParticleGetState(pVector, state);
	// get derivative
	ParticleDerivative(pVector, fVector, cVector, derivative);
	
	// Euler step
	midState.resize(state.size());
	for (size_t ii = 0; ii < state.size(); ++ii) {
		midState[ii] = state[ii] + 0.5f * deltaT * derivative[ii];
	}
	
	// set particles to midpoint state
	ParticleSetState(pVector, midState);
	
	// evaluate f at the midpoint
	ParticleDerivative(pVector, fVector, cVector, midDerivative);
	
	// take a step using the midpoint value
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += midDerivative[ii] * deltaT;
	}
	ParticleSetState(pVector, state);
}

// RK4 solver
void rk4_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector,
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, k1, k2, k3, k4;
	// get state
	ParticleGetState(pVector, state);
	
	// k1 = f(x)
	ParticleDerivative(pVector, fVector, cVector, k1);
	
	// k2 = f(x + 0.5*dt*k1)
	std::vector<float> tempState(state.size());
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + 0.5f * deltaT * k1[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k2);
	
	// k3 = f(x + 0.5*dt*k2)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + 0.5f * deltaT * k2[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k3);
	
	// k4 = f(x + dt*k3)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + deltaT * k3[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k4);
	
	// Runge-Kutta of order 4: x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += (deltaT / 6.0f) * (k1[ii] + 2.0f * k2[ii] + 2.0f * k3[ii] + k4[ii]);
	}
	ParticleSetState(pVector, state);
}

extern void simulation_step( std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector, 
	float dt )
{
	switch(solver_type)
	{
		case 0: euler_step(pVector, fVector, cVector, dt); break;
		case 1: midpoint_step(pVector, fVector, cVector, dt); break;
		case 2: rk4_step(pVector, fVector, cVector, dt); break;
	}
}

