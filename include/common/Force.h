#pragma once

#include "Particle.h"
#include <vector>

class Force {
public:
	virtual double applyForce() = 0;
	virtual void draw() = 0;
};


class SpringForce : public Force {
public:
	SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);
	double applyForce() override;
	void draw() override;

private:

	Particle * const m_p1;   // particle 1
	Particle * const m_p2;   // particle 2 
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
};

class GravityForce : public Force {
public:
	GravityForce(std::vector<Particle*> pVector);
	double applyForce() override;
	void draw() override;

private:
	std::vector<Particle*> m_pVector;
	double const m_gconstant;     // gravity constant
};


class AngularSpring : public Force {
public:
	AngularSpring(Particle* x1, Particle* x2, Particle* x3, double angle, double ks, double kd);
	double applyForce() override;
	void draw() override;

private:
	Particle* endPoint1;
	Particle* anglePoint;
	Particle* endPoint2;
	double const m_angle; // rest angle
	double const m_ks, m_kd;

	Vec2f getMidpoint();
	double distance(const Vec2f &x1, const Vec2f &x2);
	double calculate_rest_len();
	Vec2f getUnitVector(const Vec2f &x1, const Vec2f &x2);

};

double degreesToRadians(double degrees);