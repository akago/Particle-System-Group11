#include "Solver.h"
#include "util.h"
#include "ConstraintSolver.h"

void UpdateForces(std::vector<Particle*> pVector, std::vector<Force*> fVector) {
	int ii, size = pVector.size();

	// Clear forces
	for (ii = 0; ii < size; ii++) {
		pVector[ii]->clearForce();
	}
	// Compute forces
	for (auto force : fVector) {
		force->applyForce();
	}
}

void ParticleDeriv(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double* dst) {
	UpdateForces(pVector, fVector);
	ApplyConstraintForce(pVector, cVector);

	for (auto p : pVector) {
		*(dst++) = p->m_Velocity[0];
		*(dst++) = p->m_Velocity[1];
		*(dst++) = p->m_Force[0] / p->mass;
		*(dst++) = p->m_Force[1] / p->mass;
	}
}

void GetSystemState(std::vector<Particle*> pVector, double* dst) {
	for (auto p : pVector) {
		*(dst++) = p->m_Position[0];
		*(dst++) = p->m_Position[1];
		*(dst++) = p->m_Velocity[0];
		*(dst++) = p->m_Velocity[1];
	}
}

void SetSystemState(std::vector<Particle*> pVector, double* src) {
	for (auto p : pVector) {
		p->m_Position[0] = *(src++);
		p->m_Position[1] = *(src++);
		p->m_Velocity[0] = *(src++);
		p->m_Velocity[1] = *(src++);
	}
}

double* ComputeK1(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt) {
	double* k1 = (double*)malloc(pVector.size() * 4 * sizeof(double));

	// Compute k1
	ParticleDeriv(pVector, fVector, cVector, k1);
	vecTimesScalar(pVector.size() * 4, k1, dt);

	return k1;
}

double* ComputeKHelper(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt, double* k, double factor) {
	double* k_next = (double*)malloc(pVector.size() * 4 * sizeof(double));

	// Compute next k
	GetSystemState(pVector, k_next);
	vecAddEqualWithFactor(pVector.size() * 4, k_next, k, factor);

	SetSystemState(pVector, k_next);
	ParticleDeriv(pVector, fVector, cVector, k_next);
	vecTimesScalar(pVector.size() * 4, k_next, dt);

	return k_next;

}

double* ComputeK2(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt, double* k1) {
	return ComputeKHelper(pVector, fVector, cVector, dt, k1, 0.5);
}

double* ComputeK3(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt, double* k2) {
	return ComputeKHelper(pVector, fVector, cVector, dt, k2, 0.5);
}

double* ComputeK4(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt, double* k3) {
	return ComputeKHelper(pVector, fVector, cVector, dt, k3, 1);
}

void Euler_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {
	double* state = (double*)malloc(pVector.size() * 4 * sizeof(double));
	GetSystemState(pVector, state);

	double* k1 = ComputeK1(pVector, fVector, cVector, dt);
	vecAddEqual(pVector.size() * 4, state, k1);
	SetSystemState(pVector, state);
	free(state);
	free(k1);
}

void Midpoint_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {
	double* state = (double*)malloc(pVector.size() * 4 * sizeof(double));
	GetSystemState(pVector, state);

	double* k1 = ComputeK1(pVector, fVector, cVector, dt);
	double* k2 = ComputeK2(pVector, fVector, cVector, dt, k1);

	vecAddEqual(pVector.size() * 4, state, k2);
	SetSystemState(pVector, state);

	free(state);
	free(k1);
	free(k2);
}

void Runge_Kutta_4(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, float dt) {
	double* state = (double*)malloc(pVector.size() * 4 * sizeof(double));
	GetSystemState(pVector, state);

	double* k1 = ComputeK1(pVector, fVector, cVector, dt);
	double* k2 = ComputeK2(pVector, fVector, cVector, dt, k1);
	SetSystemState(pVector, state);
	double* k3 = ComputeK3(pVector, fVector, cVector, dt, k2);
	SetSystemState(pVector, state);
	double* k4 = ComputeK4(pVector, fVector, cVector, dt, k3);

	vecAddEqualWithFactor(pVector.size() * 4, state, k1, 1.0 / 6);
	vecAddEqualWithFactor(pVector.size() * 4, state, k2, 1.0 / 3);
	vecAddEqualWithFactor(pVector.size() * 4, state, k3, 1.0 / 3);
	vecAddEqualWithFactor(pVector.size() * 4, state, k4, 1.0 / 6);
	SetSystemState(pVector, state);

	free(state);
	free(k1);
	free(k2);
	free(k3);
	free(k4);
}