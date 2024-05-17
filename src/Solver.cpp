#include "Solver.h"
#include "util.h"

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

void GetGlobalVectors(std::vector<Particle*> pVector, double* q, double* Q, double* W, double* qdot) {
	for (auto particle : pVector) {
		*(q++) = particle->m_Position[0];
		*(q++) = particle->m_Position[1];
		*(Q++) = particle->m_Force[0];
		*(Q++) = particle->m_Force[1];
		*(W++) = particle->mass;
		*(W++) = particle->mass;
		*(qdot++) = particle->m_Velocity[0];
		*(qdot++) = particle->m_Velocity[1];

	}
}

void GetGlobalMatrices(std::vector<Constraint*> cVector, double* C, double* Cdot) {
	for (int ii = 0; ii < cVector.size(); ii++) {
		cVector[ii]->fillJacobBlock();
		cVector[ii]->fillJacobDotBlock();
		*(C++) = cVector[ii]->eval();
		*(Cdot++) = cVector[ii]->getTimeDeriv();
	}
}


double calculateError(int n, double* x, double* b, double* W) {
	double* temp = (double*)malloc(n * sizeof(double));
	double* res = (double*)malloc(Constraint::global_cons_num * sizeof(double));
	Constraint::GlobalJ->matTransVecMult(x, temp);
	vecElewiseProd(n,temp,W);
	Constraint::GlobalJ->matVecMult(temp, res);

	printf("¡¾calculateError¡¿==========START===============\n");
	printf("JWJtx is: ");
	printVector(Constraint::global_cons_num, res);
	printf("b is: ");
	printVector(Constraint::global_cons_num, b);
	printf("¡¾calculateError¡¿==========END===============\n");
	

	free(temp);
	free(res);
}

void ApplyConstraintForce(std::vector<Particle*> pVector, std::vector<Constraint*> cVector) {
	double epsilon = 0.000000001;
	int steps = 0;
	int dimension = pVector.size() * 2;

	double* q = (double*)malloc(dimension * sizeof(double)); // global position vector
	double* Q = (double*)malloc(dimension * sizeof(double)); // global force vector
	double* W = (double*)malloc(dimension * sizeof(double)); // mass diagnoal matrix
	double* qdot = (double*)malloc(dimension * sizeof(double)); // global velocity vector

	double* C = (double*)malloc(Constraint::global_cons_num * sizeof(double)); 
	double* Cdot = (double*)malloc(Constraint::global_cons_num * sizeof(double));
	double* b = (double*)malloc(Constraint::global_cons_num * sizeof(double));
	double* x = (double*)malloc(Constraint::global_cons_num * sizeof(double));
	
	

	printf("¡¾ApplyConstraintForce¡¿===============STARTING===================\n");

	GetGlobalVectors(pVector, q, Q, W, qdot);
	
	printf("Force vector: ");
	printVector(dimension, Q);

	// GetGlobalMatrix
	GetGlobalMatrices(cVector, C, Cdot);

	Constraint::GlobalJdot->matVecMult(qdot, b);						// Jdot*qdot
	vecElewiseProd(dimension, Q, W);									// W*Q
	Constraint::GlobalJ->matVecMult(Q, x);								// JWQ
	vecTimesScalar(Constraint::global_cons_num, C, Constraint::ks);		// ks*C
	vecTimesScalar(Constraint::global_cons_num, Cdot, Constraint::kd); // kd*Cdot
	// Sum up
	vecAddEqual(Constraint::global_cons_num, b, x);
	vecAddEqual(Constraint::global_cons_num, b, C);
	vecAddEqual(Constraint::global_cons_num, b, Cdot);
	vecTimesScalar(Constraint::global_cons_num, b, -1.0);
	
	printf("C vector: ");
	printVector(Constraint::global_cons_num, C);
	printf("Cdot vector: ");
	printVector(Constraint::global_cons_num, Cdot);

	Constraint_ConjGrad(Constraint::global_cons_num, dimension, Constraint::GlobalJ, x, b, W, epsilon, steps);
	calculateError(dimension, x, b, W);
	
	Constraint::GlobalJ->matTransVecMult(x, Q);
	printf("Lambda:");
	printVector(Constraint::global_cons_num, x);
	double* constraint_forces = Q;
	printf("Constraint force: ");
	printVector(dimension, Q);
	for (auto particle : pVector) {
		particle->m_Force[0] += *(constraint_forces++);
		particle->m_Force[1] += *(constraint_forces++);
	}
	printf("¡¾ApplyConstraintForce¡¿===============END===================\n");
	free(q);
	free(Q);
	free(W);
	free(qdot);
	free(C);
	free(Cdot);
	free(b);
	free(x);
}


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

double* ComputeK1(std::vector<Particle*> pVector, std::vector<Force*> fVector, std::vector<Constraint*> cVector, double dt) {
	UpdateForces(pVector, fVector);
	ApplyConstraintForce(pVector, cVector);

	double* k1 = (double*)malloc(pVector.size() * 4 * sizeof(double));

	// Compute k1
	ParticleDeriv(pVector, k1);
	vecTimesScalar(pVector.size() * 4, k1, dt);

	return k1;
}

double* ComputeKHelper(std::vector<Particle*> pVector, std::vector<Force*> fVector, double dt, double* k, double factor) {
	double* k_next = (double*)malloc(pVector.size() * 4 * sizeof(double));

	// Compute next k
	GetSystemState(pVector, k_next);
	vecAddEqualWithFactor(pVector.size() * 4, k_next, k, factor);

	SetSystemState(pVector, k_next);
	UpdateForces(pVector, fVector);

	ParticleDeriv(pVector, k_next);
	vecTimesScalar(pVector.size() * 4, k_next, dt);

	return k_next;

}

double* ComputeK2(std::vector<Particle*> pVector, std::vector<Force*> fVector, double dt, double* k1) {
	return ComputeKHelper(pVector, fVector, dt, k1, 0.5);
}

double* ComputeK3(std::vector<Particle*> pVector, std::vector<Force*> fVector, double dt, double* k2) {
	return ComputeKHelper(pVector, fVector, dt, k2, 0.5);
}

double* ComputeK4(std::vector<Particle*> pVector, std::vector<Force*> fVector, double dt, double* k3) {
	return ComputeKHelper(pVector, fVector, dt, k3, 1);
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
	double* k2 = ComputeK2(pVector, fVector, dt, k1);

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
	double* k2 = ComputeK2(pVector, fVector, dt, k1);
	SetSystemState(pVector, state);
	double* k3 = ComputeK3(pVector, fVector, dt, k2);
	SetSystemState(pVector, state);
	double* k4 = ComputeK4(pVector, fVector, dt, k3);

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

