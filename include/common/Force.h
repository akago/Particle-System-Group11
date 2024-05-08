#pragma once

#include "Particle.h"

class Force {
public:
	virtual void applyForce() = 0;
	virtual void draw() = 0;
};
