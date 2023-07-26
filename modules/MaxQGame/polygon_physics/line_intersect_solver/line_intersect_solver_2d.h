/* line_intersect_solver_2d.h */

#pragma once

#include "../poly_physics_2d.h"

class LineIntersectSolver2D : public PolygonPhysicsSystem2D {
	GDCLASS(LineIntersectSolver2D, PolygonPhysicsSystem2D);

protected:
	Vector2 from = { 0, 0 };
	Vector2 to = { 0, 0 };

	bool intersect_bodies = true;

public:

	bool get_solution(){};
};
