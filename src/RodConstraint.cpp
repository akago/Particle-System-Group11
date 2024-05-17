#include "Constraint.h"
#include <GL/glut.h>
RodConstraint::RodConstraint(int p1_idx, int p2_idx, Particle *p1, Particle * p2, double dist) :
 m_p1_idx(p1_idx), m_p2_idx(p2_idx), m_p1(p1), m_p2(p2), m_dist(dist), m_c_idx(global_cons_num++) 
{

}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}
