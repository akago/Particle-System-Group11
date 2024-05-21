#include "SampleSystems.h"
#include "ParticleSystem.h"
#include "ClothParticleSystem.h"

ParticleSystem* sample1() {
    ParticleSystem* system = new ParticleSystem();

    system->setDt(0.1f);

    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Set integration scheme.
	system->setIntegrationHook(RungeKutta);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

    Particle* particle0 = new Particle(center + offset);
    Particle* particle1 = new Particle(center + offset + offset);
    Particle* particle2 = new Particle(center + offset + offset + offset);

	system->addParticle(particle0);
	system->addParticle(particle1);
	system->addParticle(particle2);
	
	system->addForce(new GravityForce(system->getParticles(), 0.001));
	system->addForce(new SpringForce(particle0, particle1, dist, 0.005, 0.001));

	// Create Global Constraint Jacobian Matrix
	Constraint::GlobalJ = new GlobalMatrix(0,system->particleCount()*2);
	Constraint::GlobalJdot = new GlobalMatrix(0, system->particleCount() * 2);
	Constraint::global_cons_num = 0;
	Constraint::kd = 0.001;
	Constraint::ks = 0.002;

	system->addConstraint(new CircularWireConstraint(0, particle0, center, dist));

    return system;
}

ParticleSystem* sample2() {
    ParticleSystem* system = new ParticleSystem();

    system->setDt(0.1f);

	const Vec2f center(0.0, 0.0);

	// Set integration scheme.
	system->setIntegrationHook(RungeKutta);

    Particle* particle0 = new Particle(center);

	system->addParticle(particle0);
	
	system->addForce(new GravityForce(system->getParticles(), 0.005));

	// Create Global Constraint Jacobian Matrix
	Constraint::GlobalJ = new GlobalMatrix(0,system->particleCount()*2);
	Constraint::GlobalJdot = new GlobalMatrix(0, system->particleCount() * 2);
	Constraint::global_cons_num = 0;
	Constraint::kd = 0.001;
	Constraint::ks = 0.002;

	system->addConstraint(new LineWireConstraint(0, particle0, 1, -1, 0));

    return system;
}

ParticleSystem* cloth1() {
	double posX = -0.5;
    double posY = 0.5;
    int width = 10;
    int height = 10;
    double dist = 0.1;
    double mass = 1.0;


	double ks = 7.0;
	double kd = 3.0;

    double structural_ks = ks;
    double structural_kd = kd;
    double flexion_ks = ks;
    double flexion_kd = kd;
	double shear_ks = ks;
    double shear_kd = kd;

	double deformation_rate = 1.1;

	ClothParticleSystem* system = new ClothParticleSystem(posX, posY, width, height, dist, mass ,structural_ks, structural_kd, flexion_ks, flexion_kd, shear_ks, shear_kd, deformation_rate);

	system->fixPoint(0,0);
	system->fixPoint(9,0);
	system->fixPoint(5,0);

	system->setIntegrationHook(RungeKutta);
	system->setDt(0.01);
	system->addForce(new GravityForce(system->getParticles(), 0.05));

	return system;
}