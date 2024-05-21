#include "ClothParticleSystem.h"

int ClothParticleSystem::index(int x, int y)
{
    return y * width + x;
}

Vec2f ClothParticleSystem::pos(int x, int y)
{
    return Vec2f(posX + x * dist, posY + y * -dist);
}

ClothParticleSystem::ClothParticleSystem(double posX, double posY, int width, int height, double dist, double mass, double structural_ks, 
                                            double structural_kd, double flexion_ks, double flexion_kd, double shear_ks, double shear_kd) 
    : posX(posX), posY(posY), width(width), height(height), dist(dist)
{
    // initialize particles
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            particles.push_back(new Particle(pos(x, y), mass));
        }
    }

    // initialize forces
    // structural spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 1; x++)
        {
            forces.push_back(new SpringForce(particles[index(x, y)], particles[index(x + 1, y)], dist, structural_ks, structural_kd));
        }
    }

    for (int y = 0; y < height - 1; y++)
    {
        for (int x = 0; x < width; x++)
        {
            forces.push_back(new SpringForce(particles[index(x, y)], particles[index(x, y + 1)], dist, structural_ks, structural_kd));
        }
    }

    // flexion spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 2; x++)
        {
            forces.push_back(new SpringForce(particles[index(x, y)], particles[index(x + 2, y)], dist * 2, flexion_ks, flexion_kd));
        }
    }

    for (int y = 0; y < height - 2; y++)
    {
        for (int x = 0; x < width; x++)
        {
            forces.push_back(new SpringForce(particles[index(x, y)], particles[index(x, y + 2)], dist * 2, flexion_ks, flexion_kd));
        }
    }

    // shear spring forces
    for (int y = 0; y < height - 1; y++)
    {
        for (int x = 0; x < width - 1; x++)
        {
            forces.push_back(new SpringForce(particles[index(x, y)], particles[index(x + 1, y + 1)], dist * TWOSQUAREROOT, shear_ks, shear_kd));
            forces.push_back(new SpringForce(particles[index(x, y + 1)], particles[index(x + 1, y)], dist * TWOSQUAREROOT, shear_ks, shear_kd));
        }
    }
}

void ClothParticleSystem::setCornerConstraints()
{
    Constraint::GlobalJ = new GlobalMatrix(0, particles.size() * 2);
    Constraint::GlobalJdot = new GlobalMatrix(0, particles.size() * 2);
    Constraint::global_cons_num = 0;
    Constraint::kd = 0.0;
    Constraint::ks = 0.0;

    // constraints.push_back(new CircularWireConstraint(index(0,0), particles[index(0,0)], pos(0,0)+Vec2f(0,0.05), 0.05));
    // constraints.push_back(new CircularWireConstraint(index(width-1,0), particles[index(width-1,0)], pos(width-1,0)+Vec2f(0,0.05), 0.05));

    constraints.push_back(new CircularWireConstraint(index(0, 0), particles[index(0, 0)], pos(0, 0), 0.0));
    constraints.push_back(new CircularWireConstraint(index(width - 1, 0), particles[index(width - 1, 0)], pos(width - 1, 0), 0.0));
}

void ClothParticleSystem::simulationStep() {
    ParticleSystem::simulationStep();
}