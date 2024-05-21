#pragma once

#include "Particle.h"
#include <vector>

class Force {
public:
	virtual void applyForce() = 0;
	virtual void draw() = 0;
};


class SpringForce : public Force {
public:
	SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);
	void applyForce() override;
	void draw() override;

private:

	Particle * const m_p1;   // particle 1
	Particle * const m_p2;   // particle 2 
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
};

class GravityForce : public Force {
public:
	GravityForce(std::vector<Particle*> pVector, double gConstant = 0.05);
	void applyForce() override;
	void draw() override;

private:
	std::vector<Particle*> m_pVector;
	double const m_gconstant;     // gravity constant
};