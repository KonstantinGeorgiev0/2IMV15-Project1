#pragma once
#include "Force.h"
#include "Particle.h"

class AngularSpringForce : public Force {
public:
    AngularSpringForce(Particle* p1, Particle* p2, Particle* p3, double rest_angle, double ks, double kd);
    void apply() override;
    void draw() override;
private:
    Particle* m_p1; 
    Particle* m_p2; 
    Particle* m_p3; 
    double m_rest_angle;
    double m_ks;
    double m_kd;
};