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
	Vector<size_t> connected_collision_pair_id_vec;

	// THESE MUST GET CHANGED AT SOME POINT BY THE SEGMENT DATA GETTER
	real_t total_mass = INFINITY;
	real_t inverse_total_mass = 0;
	real_t total_inertia = INFINITY;
	real_t inverse_total_inertia = 0;

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

	void set_total_mass(real_t mass);
	void set_total_inertia(real_t inertia);

	// Filled in during calculate_data_from_segments(), used during polygon_time_data creation
	Vector<Vector<Vector2>> polygon_data_positions;
	Vector<Vector<real_t>> polygon_data_magnitudes;
	Vector<Vector<real_t>> polygon_data_angles;
	real_t biggest_magnitude; // Used for determining how many timesteps we should generate

	// The first vector represents which segment we're talking about, the second is the time instance, and the third is the points at that time
	// Vector3 because it's X, Y, and time, this format makes it easy to use in calculations even though it seems time gets duplicated a lot for no reason
	Vector<Vector<Vector<Vector3>>> polygon_spacetime_data;

public:
	Rect2 collision_bounding_box = { 0, 0, 0, 0 }; // Basically real_bounding_box + future_bounding_box
	Rect2 get_collision_box();

	real_t get_total_mass();
	real_t get_total_inertia();
	real_t get_inverse_total_mass();
	real_t get_inverse_total_inertia();

	Vector2 get_velocity();
	real_t get_angular_velocity();

	/*
	// -1 signals false, otherwise returns index of a segment which contains the provided point. It will not return EVERY segment which contains the point.
	int which_segment_contains_point(Vector2 global_point);
	// Returns how far we must displace the point in order for it to no longer be located within the entity, which you can get by
	// multiplying the direction vector with the result's param_t
	// Like its less-capable brother, it returns the index of the associated vector or -1 if the point is not in the entity.
	int get_point_displacement_vec(Vector2 global_point, Vector2 direction_vec, VectorVectorIntersectResult2D* result);
	*/

	void queue_central_force(Vector2 force);
	void queue_torque(real_t torque);

	real_t get_torque_from_force(Vector2 global_location, Vector2 force);

	void apply_force(Vector2 global_location, Vector2 force);
	void apply_central_force(Vector2 force);
	void apply_torque(real_t torque);

	void apply_impulse_force(Vector2 global_location, Vector2 impulse);
	void apply_central_impulse_force(Vector2 impulse);
	void apply_impulse_torque(real_t impulse_torque);

	Vector<PhysicsSegment2D *> get_segments();
	int add_segment(PhysicsSegment2D *physics_segment);
	void remove_segment_at(int id);
	void queue_calculate_data_from_segments(); // For performance, calculate_data_from_segments() is a hungry function
	void calculate_data_from_segments();

	void set_id(size_t index);

	// Solver and collision resolution stuff
	void solver_apply_queued_forces(real_t delta_time);
	void solver_prepare_for_step(real_t delta_time, real_t solver_linear_resolution, real_t solver_angular_resolution);

	const Vector<Vector<Vector<Vector3>>> *solver_get_spacetime_data();

	// THE BOTTOM TWO FUNCS ARE JUST OVERCOMPLICATING IT, FIX TOMORROW!
	//size_t solver_generate_spacetime_point_for_all_segments(real_t time);
	// The index is for an array which contains the generated time points in order of generation
	//size_t solver_generate_spacetime_data_from_generated_spacetime_point(size_t index);

	void solver_step_for_toi(real_t toi);
	void solver_step(real_t delta_time);

	void clear_collision_data();
	real_t get_total_toi();

	void solver_add_collision_pair(size_t pair_id);
	bool solver_collision_pair_already_exists(size_t other_body_id);
	// removes a specific collision pair without safety, use with care as other participants will not be aware of this
	// Used if something else is already safely clearing the collision pair
	void solver_remove_collision_pair(size_t pair_id);
	// Forces the collision pair vec safely, making sure other participants are cleared as well
	void solver_invalidate_collision_pair_vec();
	// Forces the entire collision pair vec to be cleaned without safety, only use when the solver's collision pair vec is being cleared as well
	void solver_clear_collision_pair_vec();

	size_t *solver_get_polygon_data_vec_size();
	Vector<Vector<Vector2>> *solver_get_direct_polygon_data_access(size_t index);

	/*
	void load_all_current_points(Vector<Vector<Vector2>> &target_vec);
	void load_all_specified_timestep_points(Vector<Vector<Vector2>> &target_vec, real_t target_time);
	void load_all_final_points(Vector<Vector<Vector2>> &target_vec);
	*/

	/*
	bool solver_find_outbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, PhysicsEntity2D *other);
	bool solver_find_inbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, Vector2 global_start_point,
															Vector2 global_end_point, real_t starting_detltatime);
	*/
};
