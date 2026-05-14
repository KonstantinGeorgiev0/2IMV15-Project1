#pragma once
#include <vector>
#include "Particle.h"

class CollisionHandler {
public:
    static void handleWallCollisions(std::vector<Particle*>& particles, float restitution);
    static void drawWalls();
};