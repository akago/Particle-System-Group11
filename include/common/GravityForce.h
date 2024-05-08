#pragma once

#include "Force.h"
#include <vector>

class GravityForce : public Force {
public:
	GravityForce(std::vector<Particle*> pVector);
	void applyForce() override;
	void draw() override;

private:
	std::vector<Particle*> m_pVector;
	double const m_gconstant;     // gravity constant
};