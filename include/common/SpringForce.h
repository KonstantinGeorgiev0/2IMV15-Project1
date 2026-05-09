#pragma once

#include "Particle.h"

#include "Force.h" // Including the base class

class SpringForce : public Force { // Inheriting from Force
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);

  void apply() override; // Adding apply method
  void draw() override;

 private:

  Particle * const m_p1;   // particle 1
  Particle * const m_p2;   // particle 2 
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
