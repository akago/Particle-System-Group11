#pragma once

#include "Particle.h"
#include <cmath>

class Constraint {
public:
	virtual double eval() = 0;
	virtual void getJacobian() = 0;
	virtual void getTimeDeriv() = 0;
	virtual void get
	virtual void draw() = 0;
};

/*
	C(x,y) = (x-xc)^2 + (y-yc)^2-r^2
	Cdot = 2(x-xc)vx + 2(y-yc)vy
*/
class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);
  double eval() override;
  void getJacobian() override;
  void getTimeDeriv() override;
  void draw() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};

/*
	Constraint Equation: C(x1,y1,x2,y2) = (x1-x2)^2 + (y1-y2)^2 - r^2
*/
class RodConstraint : public Constraint {
public:
	RodConstraint(Particle *p1, Particle * p2, double dist);
	void getJacobian() override;
	void draw();

private:

	Particle * const m_p1;
	Particle * const m_p2;
	double const m_dist;
};

/*
	Constraint Equation: C(x1,y1,x2,y2) = ¡Ì[(x1-x2)^2 + (y1-y2)^2] - r
*/
class RodConstraintv2 : public RodConstraint {
public:
	void getJacobian() override;
};

struct SparseBlock {
	int row_start;
	int col_start;
	std::vector<int> values;
};

class BlockCompressedSparseMatrix {
private:
	int rows;
	int cols;
	int block_size; // Size of each block
	std::vector<SparseBlock> blocks;

public:
	BlockCompressedSparseMatrix(int rows, int cols, int block_size)
		: rows(rows), cols(cols), block_size(block_size) {}

	void insertBlock(int row_start, int col_start, const std::vector<int>& values) {
		SparseBlock block = { row_start, col_start, values };
		blocks.push_back(block);
	}

	int getBlockValue(int row, int col) {
		int block_row = row / block_size;
		int block_col = col / block_size;

		for (const auto &block : blocks) {
			if (block.row_start <= block_row && block.col_start <= block_col &&
				block.row_start + block_size > block_row && block.col_start + block_size > block_col) {
				int inner_row = row % block_size;
				int inner_col = col % block_size;
				int index = inner_row * block_size + inner_col;
				return block.values[index];
			}
		}
		return 0; // assuming 0 for non-existing blocks
	}

	// Implement other necessary functions like matrix multiplication, addition, etc.
};