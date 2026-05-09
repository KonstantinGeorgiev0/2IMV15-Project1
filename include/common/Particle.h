#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw();
	void clearForce(); // ADDED: Function to reset force to zero

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;

	Vec2f m_Force; // ADDED this to accumulate forces
	float m_Mass;  // ADDED: Mass of the particle (i.e., default to 1.0)
};
