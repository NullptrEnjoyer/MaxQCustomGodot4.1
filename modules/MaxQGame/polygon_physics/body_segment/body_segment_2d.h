/* body_segment_2d.h */

#pragma once

#include "scene\2d\polygon_2d.h"

#include "../physics_entity/physics_entity_2d.h"

class PhysicsEntity2D; //<---- CIRCULAR DEPENDENCY

class PhysicsSegment2D : public Polygon2D {
	GDCLASS(PhysicsSegment2D, Polygon2D);

protected:
	static String PhysicsEntityTypeID;

	PhysicsEntity2D *entity = nullptr;
	int ID = 0;

	Vector2 center_of_mass = { 0, 0 };
	real_t mass = 0;
	real_t area = 0;

	static void _bind_methods();
	void _notification(int p_what);

	void try_find_physics_entity();
	void try_remove_from_physics_entity();

	void calculate_area();
	void calculate_center_of_mass();

public:
	void set_mass(real_t _mass);
	real_t get_mass();

	Vector2 get_center_of_mass();

	bool get_intersect_results_local(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset, Vector<Vector2> *results);
};
