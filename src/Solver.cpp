#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Force.h"
#include "Constraint.h"
#include "ConstraintSolver.h"
#include "linearSolver.h"

#include <vector>
#include <cmath>

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

// Implicit Euler  (Baraff-Witkin linearisation)
// Only spring forces contribute non-zero Jacobians; gravity/wind have J≈0.
// Pinned particles are enforced via a large diagonal (Δv → 0).

struct ImplicitSpringBlock {
    int pi, pj;      // particle indices
    double C[2][2];  // combined block
};

class ImplicitEulerMatrix : public implicitMatrix {
public:
    std::vector<Particle*>*         particles;
    std::vector<ImplicitSpringBlock> blocks;

    void matVecMult(double x[], double r[]) override {
        const int n = (int)particles->size();
        for (int k = 0; k < 2 * n; k++) r[k] = 0.0;

        // Mass rows
        for (int i = 0; i < n; i++) {
            double m = (*particles)[i]->m_Mass;
            r[2*i]   += m * x[2*i];
            r[2*i+1] += m * x[2*i+1];
        }

        // Spring stiffness + damping rows
        for (const auto& b : blocks) {
            double dv[2] = { x[2*b.pi]   - x[2*b.pj],
                             x[2*b.pi+1] - x[2*b.pj+1] };
            for (int a = 0; a < 2; a++) {
                double cv = b.C[a][0]*dv[0] + b.C[a][1]*dv[1];
                r[2*b.pi+a] += cv;
                r[2*b.pj+a] -= cv;
            }
        }

        // Pinned override
        for (int i = 0; i < n; i++) {
            if ((*particles)[i]->m_Pinned) {
                r[2*i]   = 1e10 * x[2*i];
                r[2*i+1] = 1e10 * x[2*i+1];
            }
        }
    }
};

// Build particle index lookup (O(1) per query after build)
static int particle_idx(Particle* p, const std::vector<Particle*>& pv) {
    for (int i = 0; i < (int)pv.size(); i++)
        if (pv[i] == p) return i;
    return -1;
}

void implicit_euler_step(std::vector<Particle*> pVector,
    std::vector<Force*> fVector,
    std::vector<Constraint*> cVector,
    float h)
{
    const int N = (int)pVector.size();

    //Accumulate forces(including constraint corrections)
    for (int i = 0; i < N; i++)
        pVector[i]->m_Force = Vec2f(0.0f, 0.0f);
    for (auto* f : fVector)
        f->apply();
    solve_constraints(pVector, cVector);

    //Build RHS and spring block list 
    std::vector<double> b(2 * N, 0.0);

    
    for (int i = 0; i < N; i++) {
        b[2*i]   = h * pVector[i]->m_Force[0];
        b[2*i+1] = h * pVector[i]->m_Force[1];
    }

    ImplicitEulerMatrix A;
    A.particles = &pVector;

    for (auto* force : fVector) {
        SpringForce* sf = dynamic_cast<SpringForce*>(force);
        if (!sf) continue;

        Particle* pa = sf->getP1();
        Particle* pb = sf->getP2();
        int pi = particle_idx(pa, pVector);
        int pj = particle_idx(pb, pVector);
        if (pi < 0 || pj < 0) continue;

        float ks = (float)sf->getKs();
        float kd = (float)sf->getKd();
        float r  = (float)sf->getDist();

        Vec2f l   = pa->m_Position - pb->m_Position;
        float len = sqrtf(l[0]*l[0] + l[1]*l[1]);
        if (len < 1e-8f) continue;

        Vec2f lhat = l / len;

        double lhat_outer[2][2] = {
            { lhat[0]*lhat[0], lhat[0]*lhat[1] },
            { lhat[1]*lhat[0], lhat[1]*lhat[1] }
        };

        double perp = std::max(0.0, (double)(len - r) / len);
        ImplicitSpringBlock blk;
        blk.pi = pi;
        blk.pj = pj;
        for (int a = 0; a < 2; a++) {
            for (int bb = 0; bb < 2; bb++) {
                double I_ab = (a == bb) ? 1.0 : 0.0;
                double J_stiff = ks * (lhat_outer[a][bb] + perp * (I_ab - lhat_outer[a][bb]));
                double J_damp  = kd * lhat_outer[a][bb];
                blk.C[a][bb] = h*h * J_stiff + h * J_damp;
            }
        }
        A.blocks.push_back(blk);

        Vec2f v_rel = pa->m_Velocity - pb->m_Velocity;
        for (int a = 0; a < 2; a++) {
            double kv = 0.0;
            for (int bb = 0; bb < 2; bb++) {
                double I_ab = (a == bb) ? 1.0 : 0.0;
                double J_stiff = ks * (lhat_outer[a][bb] + perp * (I_ab - lhat_outer[a][bb]));
                kv -= J_stiff * v_rel[bb];
            }
            b[2*pi+a] += h*h * kv;
            b[2*pj+a] -= h*h * kv;
        }
    }

    // Zero out pinned particles' RHS
    for (int i = 0; i < N; i++) {
        if (pVector[i]->m_Pinned) {
            b[2*i] = 0.0;
            b[2*i+1] = 0.0;
        }
    }

    std::vector<double> dv(2 * N, 0.0);
    int steps = 2 * N;  
    ConjGrad(2 * N, &A, dv.data(), b.data(), 1e-6, &steps);

    for (int i = 0; i < N; i++) {
        if (pVector[i]->m_Pinned) continue;
        pVector[i]->m_Velocity[0] += (float)dv[2*i];
        pVector[i]->m_Velocity[1] += (float)dv[2*i+1];
        pVector[i]->m_Position[0] += h * pVector[i]->m_Velocity[0];
        pVector[i]->m_Position[1] += h * pVector[i]->m_Velocity[1];
    }

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
		case 3: implicit_euler_step(pVector, fVector, cVector, dt); break;
	}
}
