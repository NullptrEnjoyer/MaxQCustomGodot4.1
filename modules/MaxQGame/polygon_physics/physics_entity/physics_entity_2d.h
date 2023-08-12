/* physics_entity_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../independent_solver/independent_physics_solver_2d.h"
#include "../body_segment/body_segment_2d.h"

class IndependentPhysicsSolver2D; //<---- CIRCULAR DEPENDENCY
class PhysicsSegment2D; //<---- CIRCULAR DEPENDENCY

class PhysicsEntity2D : public PolygonPhysicsSystem2D {
	GDCLASS(PhysicsEntity2D, PolygonPhysicsSystem2D);

protected:
	static String PhysicsSolverTypeID;

	IndependentPhysicsSolver2D *solver = nullptr;
	int physics_id = 0; // Our index in the current solver. Check if solver is nullptr before using
	
	real_t total_toi = 0; // Time of impact counter used in continous collision detection
	Vector<real_t> connected_collision_pair_id_vec;

	real_t total_mass = 0;
	real_t total_inertia = 0;

	Vector2 queued_force = { 0, 0 };
	real_t queued_torque = 0.0;

	Vector2 velocity = { 0, 0 };
	real_t angular_velocity = 0.0;

	Vector<PhysicsSegment2D *> physics_segments;
	bool calculate_data_from_segments_next_step = false;
	
	Rect2 local_bounding_box = { 0, 0, 0, 0 };
	Rect2 real_bounding_box = { 0, 0, 0, 0 };
	Rect2 future_bounding_box = { 0, 0, 0, 0 };

	void _notification(int p_what);
	static void _bind_methods();

	void force_to_velocity(Vector2 force);
	void torque_to_angular_velocity(real_t torque);

	void try_find_physics_controller();
	void try_remove_from_physics_controller();

public:
	Rect2 collision_bounding_box = { 0, 0, 0, 0 }; // Basically real_bounding_box + future_bounding_box
	Rect2 get_collision_box();

	Vector2 get_velocity();
	real_t get_angular_velocity();

	void queue_central_force(Vector2 force);
	void queue_torque(real_t torque);

	Vector<PhysicsSegment2D *> get_segments();
	int add_segment(PhysicsSegment2D *physics_segment);
	void remove_segment_at(int id);
	void queue_calculate_data_from_segments(); // For performance, it's a hungry function
	void calculate_data_from_segments();

	void set_id(size_t index);

	// Solver and collision resolution stuff
	void solver_apply_queued_forces();
	void solver_prepare_for_step();
	void solver_step_for_toi(real_t toi);
	void solver_step();

	void clear_collision_data();
	void clear_segment_position_data();
	real_t get_total_toi();

	void solver_add_collision_pair(size_t pair_id);
	bool solver_collision_pair_already_exists(size_t other_body_id);
	void solver_remove_collision_pair(size_t pair_id);
	void solver_invalidate_collision_pair_vec();
	void solver_clear_collision_pair_vec();

	void load_all_current_points(Vector<Vector<Vector2>> &target_vec);
	void load_all_specified_timestep_points(Vector<Vector<Vector2>> &target_vec, real_t target_time);
	void load_all_final_points(Vector<Vector<Vector2>> &target_vec);

	/*
	bool solver_find_outbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, PhysicsEntity2D *other);
	bool solver_find_inbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, Vector2 global_start_point,
															Vector2 global_end_point, real_t starting_detltatime);*/
};
