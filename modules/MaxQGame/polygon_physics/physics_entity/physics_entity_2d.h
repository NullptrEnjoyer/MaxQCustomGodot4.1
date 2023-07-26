/* physics_entity_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../threaded_solver/threaded_physics_solver_2d.h"
#include "../body_segment/body_segment_2d.h"

class ThreadedPhysicsSolver2D; //<---- CIRCULAR DEPENDENCY
class PhysicsSegment2D; //<---- CIRCULAR DEPENDENCY

class PhysicsEntity2D : public PolygonPhysicsSystem2D {
	GDCLASS(PhysicsEntity2D, PolygonPhysicsSystem2D);

protected:
	static String PhysicsSolverTypeID;

	ThreadedPhysicsSolver2D *solver = nullptr;
	int ID = 0;

	real_t total_mass = 0;
	real_t total_inertia = 0;

	Vector2 relative_pos = { 0, 0 }; // Min x and y relative to centre of mass
	Vector2 size = { 0, 0 }; // Max x and y (relative to Min x and y)

	Vector2 safe_relative_pos = { 0, 0 }; // Min * 2
	Vector2 safe_size = { 0, 0 }; // Size * 2

	Vector2 queued_force = { 0, 0 };
	real_t queued_torque = 0.0;

	Vector2 velocity = { 0, 0 };
	real_t angular_velocity = 0.0;

	Vector<PhysicsSegment2D *> physics_segments;
	bool calculate_data_from_segments_next_step = false;

	void _notification(int p_what);

	void force_to_velocity(Vector2 force);
	void torque_to_angular_velocity(real_t torque);

	void try_find_physics_controller();
	void try_remove_from_physics_controller();

public:

	void queue_central_force(Vector2 force);
	void queue_torque(real_t torque);

	void apply_queued_forces();
	void check_collision(PhysicsEntity2D *other){};
	void step();

	int add_segment(PhysicsSegment2D *physics_segment);
	void remove_segment_at(int id);
	void queue_calculate_data_from_segments();
	void calculate_data_from_segments();

	void compute_raycasts(){};

};
