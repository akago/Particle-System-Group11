#include "ParticleSystem.h";

ParticleSystem::ParticleSystem() {
    integrationMethod = Euler_step;
}

/*
----------------------------------------------------------------------
Set hook function -- plugable integration scheme
Euler: Euler step
Midpoint: Midpoint step
RungeKutta: RungeKutta-4 step
----------------------------------------------------------------------
*/
void ParticleSystem::setIntegrationHook(IntegrationType t) {
    switch (t)
	{
		case Euler:
			integrationMethod = Euler_step;
			break;
		case Midpoint:
			integrationMethod = Midpoint_step;
			break;
		case RungeKutta:
			integrationMethod = Runge_Kutta_4;
			break;
		default:
			integrationMethod = Euler_step; //default approach
			break;
	}
}

void ParticleSystem::setDt(float dt) {
    this->dt = dt;
}

void ParticleSystem::simulationStep() {
    integrationMethod(particles, forces, constraints, dt);
}

void ParticleSystem::addParticle(Particle* particle) {
    particles.push_back(particle);
}

void ParticleSystem::addForce(Force* force) {
    forces.push_back(force);
}

void ParticleSystem::addConstraint(Constraint* constraint) {
    constraints.push_back(constraint);
}

std::vector<Particle*>& ParticleSystem::getParticles() {
    return particles;
}
std::vector<Force*>& ParticleSystem::getForces() {
    return forces;
}
std::vector<Constraint*>& ParticleSystem::getConstraints() {
    return constraints;
}

int ParticleSystem::particleCount() {
    return particles.size();
}

void ParticleSystem::drawParticles ( )
{
    for (auto particle : particles) {
		particle->draw();
	}
}

void ParticleSystem::drawForces ( )
{
	for (auto force : forces) {
		force->draw();
	}
		
}

void ParticleSystem::drawConstraints ( )
{
	for (auto constraint : constraints) {
		constraint->draw();
	}
}

void ParticleSystem::reset() {
    for (auto particle : particles) {
		particle->reset();
	}
}

ParticleSystem::~ParticleSystem() {
    particles.clear();
    forces.clear();
    constraints.clear();
}