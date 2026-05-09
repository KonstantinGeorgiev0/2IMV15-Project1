//
// Created by Cristiana Carbunaru on 09/05/2026.
//

#include "GravityForce.h"

GravityForce::GravityForce(const std::vector<Particle*>& particles, const Vec2f& gravity)
    : m_particles(particles), m_gravity(gravity) {}

void GravityForce::apply() {
    for (Particle* p : m_particles) {
        // i.e., F = m * g
        p->m_Force += p->m_Mass * m_gravity;
    }
}

void GravityForce::draw() {
    // NB: Gravity is invisible, so we don't draw anything per se.
}