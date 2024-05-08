#pragma once

#include "Particle.h"
#include <vector>
#include "Force.h"

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)

enum IntegrationType
{
	Euler,
	Midpoint,
	RungeKutta
};

typedef void(*IntegrationFunctionHook)(std::vector<Particle*>, std::vector<Force*> fVector, float);

void ParticleDeriv(std::vector<Particle*> pVector, double* dst);
void GetSystemState(std::vector<Particle*> pVector, double* dst);
void SetSystemState(std::vector<Particle*> pVector, double* src);

void Euler_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt);
void Midpoint_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt);
void Runge_Kutta_4(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt);
