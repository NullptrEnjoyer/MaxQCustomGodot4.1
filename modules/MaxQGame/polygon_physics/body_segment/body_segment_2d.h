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

	// The coefficient of restitution is (will be) calculated as (bouncyness1 + bouncyness2) / 2
	real_t bouncyness = 0.01;

	Vector2 center_of_mass = { 0, 0 };
	real_t mass = 1;
	real_t area = 0;

	bool currpos_generated = false;
	Vector<Vector2> currpos;
	bool nextpos_generated = false;
	real_t generated_nextpos_for_time = 0;// Just in case
	Vector<Vector2> nextpos;

	static void _bind_methods();
	void _notification(int p_what);

	void try_find_physics_entity();
	void try_remove_from_physics_entity();
	void try_refresh_physics_entity();

	void calculate_area();
	void calculate_center_of_mass();

public:
	void set_mass(real_t _mass);
	real_t get_mass();
	real_t get_area();

	real_t get_bouncyness();
	void set_bouncyness(real_t new_value);

	Vector2 get_center_of_mass();

	/*
	// These are meant to only be called by the physics entity during the physics tick. Currpos also gets called by PhysicsEntity2D::which_segment_contains_point
	// 
	// Tries to get the current position. Will generate one and save it if it hasn't been generated yet.
	Vector<Vector2> entity_get_global_currpos();
	//Generates specified position based on the vector from currpos to nextpos. Will not save it
	Vector<Vector2> entity_get_global_thispos(real_t target_time);
	// Tries to get the next position. Will generate one and save it if it hasn't been generated yet.
	Vector<Vector2> entity_get_global_nextpos();

	void entity_clear_position_data();
	*/
};
