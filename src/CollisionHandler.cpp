#include "CollisionHandler.h"
#include <GLUT/glut.h>

void CollisionHandler::handleWallCollisions(std::vector<Particle*>& particles, float restitution) {
    Vec2f floor_P(0.0f, -0.9f); // point on plane
    Vec2f floor_N(0.0f, 1.0f); // floor normal
    // static double accumulated_time = 0.0; // for periodic printing
    // const double print_interval = 1.0; // print every 1 second
    // accumulated_time += 0.01;
    
    for (Particle* p : particles) {
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
            // corrected vel
            p->m_Velocity = v_t - restitution * v_n;
        
            // printf("\n--- Collision Info ---\n");
            // printf("Particle Pos after: (%f, %f)\n", p->m_Position[0], p->m_Position[1]);
            // printf("Particle Vel after: (%f, %f)\n", p->m_Velocity[0], p->m_Velocity[1]);
            // printf("COLLISION! Pos corrected to (%f, %f), Vel corrected to (%f, %f)\n", p->m_Position[0], p->m_Position[1], p->m_Velocity[0], p->m_Velocity[1]);
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