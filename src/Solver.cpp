#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
#include "ConstraintSolver.h"

#include <vector>

void simulation_step(std::vector<Particle *> pVector,
                     std::vector<Force *> forces,
                     std::vector<Constraint *> constraints,
                     float dt)
{
    int size = pVector.size();

    for (int i = 0; i < size; i++)
    {
        pVector[i]->clearForce();
    }

    for (Force *f : forces)
    {
        f->apply();
    }

    solve_constraints(pVector, constraints);

    for (int i = 0; i < size; i++)
    {
        Particle *p = pVector[i];
        Vec2f acceleration = p->m_Force / p->m_Mass;
        p->m_Velocity += dt * acceleration;
        p->m_Position += dt * p->m_Velocity;
    }
}