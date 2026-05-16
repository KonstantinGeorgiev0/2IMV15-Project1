#include "CollisionHandler.h"
#include <GLUT/glut.h>

void CollisionHandler::handleWallCollisions(std::vector<Particle*>& particles, float restitution, float friction) {
    Vec2f floor_P(0.0f, -0.9f); // point on plane
    Vec2f floor_N(0.0f, 1.0f); // floor normal
    // static double accumulated_time = 0.0; // for periodic printing
    // const double print_interval = 1.0; // print every 1 second
    // accumulated_time += 0.01;
    
    for (Particle* p : particles) {
        if (p->m_Pinned) continue;  // pinned anchors must not move by collisions
        Vec2f X = p->m_Position;

        // dist from particle to plane
        float dist = (X[0] - floor_P[0]) * floor_N[0] + (X[1] - floor_P[1]) * floor_N[1];

        // // every sec message
        // if (accumulated_time >= print_interval) {
        //     accumulated_time = 0.0; // reset
        //     printf("\n--- Collision Info ---\n");
        //     printf("Checking particle at (%f, %f): distance to floor = %f\n", X[0], X[1], dist);
        // }

        // hit
        if (dist < 0.0f) {
            // pos correction
            p->m_Position[0] = X[0] - (dist * floor_N[0]);
            p->m_Position[1] = X[1] - (dist * floor_N[1]);
            // vel correction
            Vec2f v = p->m_Velocity;
            // normal vel component
            float v_dot_n = v[0] * floor_N[0] + v[1] * floor_N[1];
            Vec2f v_n(v_dot_n * floor_N[0], v_dot_n * floor_N[1]);
            // tangential vel component
            Vec2f v_t(v[0] - v_n[0], v[1] - v_n[1]);
            // friction to tangential component
            v_t[0] *= (1.0f - friction);
            v_t[1] *= (1.0f - friction);
            // corrected vel
            p->m_Velocity = v_t - restitution * v_n;
        
            // printf("\n--- Collision Info ---\n");
            // printf("Particle Pos after: (%f, %f)\n", p->m_Position[0], p->m_Position[1]);
            // printf("Particle Vel after: (%f, %f)\n", p->m_Velocity[0], p->m_Velocity[1]);
            // printf("COLLISION! Pos corrected to (%f, %f), Vel corrected to (%f, %f)\n", p->m_Position[0], p->m_Position[1], p->m_Velocity[0], p->m_Velocity[1]);
        }
    }
}

void CollisionHandler::handleParticleCollisions(std::vector<Particle*>& particles, float particle_diameter, float restitution) {
    const float particle_diameter_sq = particle_diameter * particle_diameter;

    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            Particle* p1 = particles[i];
            Particle* p2 = particles[j];
            if (p1->m_Pinned || p2->m_Pinned) continue;  // never disturb pinned anchors

            Vec2f delta = p2->m_Position - p1->m_Position;
            float dist_sq = delta[0] * delta[0] + delta[1] * delta[1];

            if (dist_sq < particle_diameter_sq) {
                float dist = sqrt(dist_sq);
                // Avoid division by zero
                if (dist == 0.0f) {
                    dist = 0.001f;
                    delta = Vec2f(0.001f, 0.0f); // arbitrary direction
                }
                Vec2f collision_normal = delta / dist;

                // Pos correction
                float penetration_depth = particle_diameter - dist;
                Vec2f correction = collision_normal * (penetration_depth / 2.0f);
                p1->m_Position -= correction;
                p2->m_Position += correction;

                // Vel correction
                Vec2f relative_velocity = p2->m_Velocity - p1->m_Velocity;
                float vel_along_normal = relative_velocity[0] * collision_normal[0] + relative_velocity[1] * collision_normal[1];

                if (vel_along_normal > 0) continue; // already separating

                float impulse_scalar = -(1 + restitution) * vel_along_normal / 2.0f; // divide by 2 for equal mass
                Vec2f impulse = impulse_scalar * collision_normal;

                p1->m_Velocity -= impulse;
                p2->m_Velocity += impulse;
            }
        }
    }
}

void CollisionHandler::drawWalls() {
    // draw floor
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); 
    glVertex2f(-1.0f, -0.92f); 
    glVertex2f(1.0f, -0.92f);
    glEnd();
}