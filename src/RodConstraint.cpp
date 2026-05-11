#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

double RodConstraint::C() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  return d * d - m_dist * m_dist;
}

double RodConstraint::C_dot() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  Vec2f v_rel = m_p1->m_Velocity - m_p2->m_Velocity;
  return 2.0 * (d * v_rel);
}

std::vector<Particle *> RodConstraint::getParticles() const
{
  return {m_p1, m_p2};
}

std::vector<Vec2f> RodConstraint::J_rows() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  return {2.0f * d, -2.0f * d};
}

std::vector<Vec2f> RodConstraint::J_dot_rows() const
{
  Vec2f v_rel = m_p1->m_Velocity - m_p2->m_Velocity;
  return {2.0f * v_rel, -2.0f * v_rel};
}