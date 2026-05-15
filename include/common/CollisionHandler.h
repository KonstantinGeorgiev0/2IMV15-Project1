#pragma once
#include <vector>
#include "Particle.h"

class CollisionHandler {
public:
    static void handleWallCollisions(std::vector<Particle*>& particles, float restitution, float friction);
    static void handleParticleCollisions(std::vector<Particle*>& particles, float particle_diameter, float restitution);
    // static void drawParticleCollisions(std::vector<Particle*>& particles, float particle_diameter);
    static void drawWalls();
};