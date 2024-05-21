#include "ClothParticleSystem.h"
#include <cmath>

int ClothParticleSystem::index(int x, int y)
{
    return y * width + x;
}

Vec2f ClothParticleSystem::pos(int x, int y)
{
    return Vec2f(posX + x * dist, posY + y * -dist);
}

ClothParticleSystem::ClothParticleSystem(double posX, double posY, int width, int height, double dist, double mass, double structural_ks, 
                                            double structural_kd, double flexion_ks, double flexion_kd, double shear_ks, double shear_kd, double deformation_rate) 
    : posX(posX), posY(posY), width(width), height(height), dist(dist), deformationRate(deformation_rate)
{
    // initialize particles
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            particles.push_back(new Particle(pos(x, y), mass));
            isConstrained.push_back(0);
        }
    }

    // initialize forces
    // structural spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 1; x++)
        {
            SpringForce* force = new SpringForce(particles[index(x, y)], particles[index(x + 1, y)], dist, structural_ks, structural_kd);
            forces.push_back(force);
        }
    }

    for (int y = 0; y < height - 1; y++)
    {
        for (int x = 0; x < width; x++)
        {
            SpringForce* force = new SpringForce(particles[index(x, y)], particles[index(x, y + 1)], dist, structural_ks, structural_kd);
            forces.push_back(force);
        }
    }

    // flexion spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 2; x++)
        {
            SpringForce* force = new SpringForce(particles[index(x, y)], particles[index(x + 2, y)], dist * 2, flexion_ks, flexion_kd);
            forces.push_back(force);
        }
    }

    for (int y = 0; y < height - 2; y++)
    {
        for (int x = 0; x < width; x++)
        {
            SpringForce* force = new SpringForce(particles[index(x, y)], particles[index(x, y + 2)], dist * 2, flexion_ks, flexion_kd);
            forces.push_back(force);
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

void ClothParticleSystem::fixTopCorners()
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

    isConstrained[index(0, 0)] = 1;
    isConstrained[index(width - 1, 0)] = 1;
}

void ClothParticleSystem::fixTopCornersToLine() 
{
    Constraint::GlobalJ = new GlobalMatrix(0, particles.size() * 2);
    Constraint::GlobalJdot = new GlobalMatrix(0, particles.size() * 2);
    Constraint::global_cons_num = 0;
    Constraint::kd = 0.001;
    Constraint::ks = 0.002;

    // constraints.push_back(new CircularWireConstraint(index(0,0), particles[index(0,0)], pos(0,0)+Vec2f(0,0.05), 0.05));
    // constraints.push_back(new CircularWireConstraint(index(width-1,0), particles[index(width-1,0)], pos(width-1,0)+Vec2f(0,0.05), 0.05));

    constraints.push_back(new LineWireConstraint(index(0, 0), particles[index(0, 0)], 0, 1, pos(0,0)[1]));
    constraints.push_back(new LineWireConstraint(index(width - 1, 0), particles[index(width - 1, 0)], 0, 1, pos(width - 1,0)[1]));

    isConstrained[index(0, 0)] = 1;
    isConstrained[index(width - 1, 0)] = 1;
}


void ClothParticleSystem::AdjustElongatedParticles(int p_idx0, int p_idx1, double maxDistance) {
    Vec2f directionVector = ((particles[p_idx1]->m_Position)-(particles[p_idx0]->m_Position));
    double distance = sqrt(pow(directionVector[0],2) + pow(directionVector[1],2));

    if(distance > maxDistance && !(isConstrained[p_idx0] && isConstrained[p_idx1])) {
        directionVector = directionVector/distance;

        if(isConstrained[p_idx0]) {
            particles[p_idx1]->m_Position = particles[p_idx1]->m_Position - ((distance-maxDistance)*directionVector);
        }
        else if (isConstrained[p_idx1]) {
            particles[p_idx0]->m_Position = particles[p_idx0]->m_Position + ((distance-maxDistance)*directionVector);
        }
        else {
            particles[p_idx1]->m_Position = particles[p_idx1]->m_Position - (0.5*(distance-maxDistance)*directionVector);
            particles[p_idx0]->m_Position = particles[p_idx0]->m_Position + (0.5*(distance-maxDistance)*directionVector);
        }
    }
}

void ClothParticleSystem::simulationStep() {
    ParticleSystem::simulationStep();

    // adjust too elongated springs

    // structural spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 1; x++)
        {
            AdjustElongatedParticles(index(x, y),index(x+1, y),deformationRate*dist);
        }
    }

    for (int y = 0; y < height - 1; y++)
    {
        for (int x = 0; x < width; x++)
        {
            AdjustElongatedParticles(index(x, y),index(x, y+1),deformationRate*dist);
        }
    }

    // flexion spring forces
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width - 2; x++)
        {
            AdjustElongatedParticles(index(x, y),index(x+2, y),deformationRate*dist*2);
        }
    }

    for (int y = 0; y < height - 2; y++)
    {
        for (int x = 0; x < width; x++)
        {
            AdjustElongatedParticles(index(x, y),index(x, y+2),deformationRate*dist*2);
        }
    }
}

ClothParticleSystem::~ClothParticleSystem() {
    isConstrained.clear();
}