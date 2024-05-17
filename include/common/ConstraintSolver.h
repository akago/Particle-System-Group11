#pragma once

#include "Particle.h"
#include "Constraint.h"

#include <vector>

void ApplyConstraintForce(std::vector<Particle*> pVector, std::vector<Constraint*> cVector);