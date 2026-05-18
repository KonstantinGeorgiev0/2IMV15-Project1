#include "AngularSpringForce.h"
#include <cmath>
#include <GLUT/glut.h>

AngularSpringForce::AngularSpringForce(Particle* p1, Particle* p2, Particle* p3, double rest_angle, double ks, double kd)
    : m_p1(p1), m_p2(p2), m_p3(p3), m_rest_angle(rest_angle), m_ks(ks), m_kd(kd) {}

void AngularSpringForce::apply() {
    Vec2f v1 = m_p1->m_Position - m_p2->m_Position;
    Vec2f v2 = m_p3->m_Position - m_p2->m_Position;

    float l1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    float l2 = sqrt(v2[0]*v2[0] + v2[1]*v2[1]);

    // avoid division by zero
    if (l1 < 1e-6f || l2 < 1e-6f) return;

    // calculate current angle using atan2
    float a1 = atan2(v1[1], v1[0]);
    float a2 = atan2(v2[1], v2[0]);
    float current_angle = a2 - a1;
    
    float angle_diff = current_angle - m_rest_angle;

    // normalize diff to [-PI, PI] to take the shortest angular path
    while (angle_diff <= -M_PI) angle_diff += 2.0 * M_PI;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    
    // calculate perpendicular normal vectors for force application
    Vec2f n1(-v1[1]/l1, v1[0]/l1); 
    Vec2f n2(-v2[1]/l2, v2[0]/l2);

    // calculate relative angular velocity for damping
    Vec2f vel1 = m_p1->m_Velocity - m_p2->m_Velocity;
    Vec2f vel2 = m_p3->m_Velocity - m_p2->m_Velocity;
    
    float w1 = (vel1[0]*n1[0] + vel1[1]*n1[1]) / l1;
    float w2 = (vel2[0]*n2[0] + vel2[1]*n2[1]) / l2;
    float current_w = w2 - w1;

    // Hooke's law for angular torque: T = -ks * theta - kd * angular_velocity
    float torque = -m_ks * angle_diff - m_kd * current_w;

    // convert torque to linear forces : Force = Torque / Radius
    Vec2f f1 = n1 * (-torque / l1); 
    Vec2f f3 = n2 * (torque / l2);

    // apply forces
    m_p1->m_Force += f1;
    m_p3->m_Force += f3;
    m_p2->m_Force -= (f1 + f3);
}

void AngularSpringForce::draw() {
    // Draw a curved line or different color segment to indicate the angular spring is active
    glBegin(GL_LINES);
    glColor3f(0.8, 0.5, 0.5);
    glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
    glVertex2f(m_p3->m_Position[0], m_p3->m_Position[1]);
    glEnd();
}