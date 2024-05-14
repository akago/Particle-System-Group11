#include "Solver.h"
#include "linearSolver.h"

void ParticleDeriv(std::vector<Particle*> pVector, double* dst) {
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

void ConstraintForceEval(std::vector<Particle*> pVector, double* dst) {

}


void Euler_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) {
	double* derivative = (double*)malloc(pVector.size() * 4 * sizeof(double));
	double* x0 = (double*)malloc(pVector.size() * 4 * sizeof(double));
	int ii, size = pVector.size();

	// Clear forces
	for (ii = 0; ii < size; ii++) {
		pVector[ii]->clearForce();
	}
	// Compute forces
	for (auto force : fVector) {
		force->applyForce();
	}

	// Integration step
	ParticleDeriv(pVector, derivative);
	vecTimesScalar(pVector.size() * 4, derivative, dt);
	GetSystemState(pVector, x0);
	vecAddEqual(pVector.size() * 4, derivative, x0);
	SetSystemState(pVector, derivative);
	
	/*pVector[1]->m_Position[0] = pVector[1]->m_ConstructPos[0];
	pVector[1]->m_Position[1] = pVector[1]->m_ConstructPos[1];*/
	free(derivative);
	free(x0);
}

void Midpoint_step(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) {
	double* derivative = (double*)malloc(pVector.size() * 4 * sizeof(double));
	double* x0 = (double*)malloc(pVector.size() * 4 * sizeof(double));
	int ii, size = pVector.size();

	// Clear forces
	for (ii = 0; ii < size; ii++) {
		pVector[ii]->clearForce();
	}
	// Compute forces
	for (auto force : fVector) {
		force->applyForce();
	}

	// Integration step
	// first derivative
	GetSystemState(pVector, x0);
	ParticleDeriv(pVector, derivative);
	vecTimesScalar(pVector.size() * 4, derivative, dt / 2);
	vecAddEqual(pVector.size() * 4, derivative, x0);
	SetSystemState(pVector, derivative);
	// second derivative
	ParticleDeriv(pVector, derivative);
	vecTimesScalar(pVector.size() * 4, derivative, dt);
	vecAddEqual(pVector.size() * 4, derivative, x0);
	SetSystemState(pVector, derivative);

	free(derivative);
	free(x0);
}

void Runge_Kutta_4(std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt) {

}



//void simulation_step( std::vector<Particle*> pVector, std::vector<Force*> fVector, float dt )
//{
//	
//
//	
//	integration_step(pVector, dt);
//
//	//for(ii=0; ii<size; ii++)
//	//{
//	//	pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;
//	//	pVector[ii]->m_Velocity = DAMP*pVector[ii]->m_Velocity + Vec2f(RAND,RAND) * 0.005;
//	//}
//
//}


