#pragma once

#include "ParticleSystem.h"
#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

#include <vector>
#include <gfx/vec2.h>

const double TWOSQUAREROOT = 1.414213562373095;

class ClothParticleSystem : ParticleSystem {
public:
    ClothParticleSystem(double posX, double posY, int width, int height, double dist, double mass, double structural_ks, double structural_kd, double flexion_ks, double flexion_kd, double shear_ks, double shear_kd);
    void setCornerConstraints();

    void simulationStep() override;
private:
    double const posX;
    double const posY;
    int const width;
    int const height;
    double const dist;

    int index(int x, int y);
    Vec2f pos(int x, int y);
};