#include "Force.h"

GravityForce::GravityForce(std::vector<Particle*> pVector) :
	m_pVector(pVector), m_gconstant(0.000001) {}

void GravityForce::applyForce() {
	for (auto p : m_pVector) {
		p->m_Force[1] -= p->mass * m_gconstant;
	}
}

void GravityForce::draw() {

}

