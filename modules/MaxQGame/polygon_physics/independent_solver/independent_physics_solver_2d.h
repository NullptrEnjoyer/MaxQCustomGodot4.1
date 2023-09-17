/* independent_physics_solver_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../physics_entity/physics_entity_2d.h"
#include "../body_segment/body_segment_2d.h"
#include "../poly_physics_2d_defines.h"

#ifdef POLYPHYSICS_SHOULD_VISUALIZE
#include "scene/2d/line_2d.h"
#include "core\math\color.h"
#include "scene\gui\label.h"
#endif

#include "../../defines.h"

class PhysicsEntity2D; //<---- CIRCULAR DEPENDENCY
class PhysicsSegment2D; //<---- CIRCULAR DEPENDENCY

// Since I'm using 2D polygons, there is always an "intruding" body and a "defending" body.
// The intruding body is the body whose point is touching the defending body's line
struct CollisionInfo {
	size_t intruding_body_segment = 0;
	size_t intruding_body_point = 0;
	size_t defending_body_segment = 0;
	size_t defending_body_line_point_1 = 0;
	size_t defending_body_line_point_2 = 0;
};

struct CollisionPair {
	size_t ID = 0;
	real_t initial_toi = 0;
	real_t toi = 0;

	// The indexes for the affected bodies within our entities vector.
	// Segments are also indexes, used in resolution as they can have very different properties
	size_t body1 = 0;
	size_t body2 = 0;

	bool body2_is_intruding_body = false; // If false, body1 is the intruding body

	CollisionInfo additional_info; // Information which we don't get from LinePlaneIntersectResult3D

	Vector2 impact_point = { 0, 0 };
	Vector2 impact_normal = { 0, 0 };

	CollisionPair(){};
	CollisionPair(size_t _id, size_t _body1, size_t _body2, real_t body1_toi, real_t body2_toi){
		ID = _id;
		body1 = _body1;
		body2 = _body2;

		if (body1_toi > body2_toi) {
			initial_toi = body1_toi;
		} else {
			initial_toi = body2_toi;
		}
	};
};

class IndependentPhysicsSolver2D : public PolygonPhysicsSystem2D {
	GDCLASS(IndependentPhysicsSolver2D, PolygonPhysicsSystem2D);

protected:

	//static void _bind_methods();
	void _notification(int p_what);
	void tick();

	Vector<PhysicsEntity2D *> entity_vec;
	Vector<CollisionPair> collision_pair_vec;
	Vector<size_t> collision_pair_id_vec;

	real_t solver_physics_time = 0;
	real_t solver_angular_resolution = 4; // How many additional points does a point with distance from mass of 1 m rotating 1 PI/step generate? 
	real_t solver_linear_resolution = 0.5; // How many additional points does a body moving by 1m generate?

public:
#ifdef POLYPHYSICS_SHOULD_VISUALIZE
	Vector2 rep_collision_point = { 0, 0 };
	Vector2 rep_collision_normal = { 0, 0 };

	Vector2 rep_intruding_body_center = { 0, 0 };
	Vector2 rep_intruding_body_velocity = { 0, 0 };
	real_t rep_intruding_body_angular_velocity = 0;
	Vector2 rep_intruding_body_momentum = { 0, 0 };
	real_t rep_intruding_body_angular_momentum = 0;

	Vector2 rep_defending_body_center = { 0, 0 };
	Vector2 rep_defending_body_velocity = { 0, 0 };
	real_t rep_defending_body_angular_velocity = 0;
	Vector2 rep_defending_body_momentum = { 0, 0 };
	real_t rep_defending_body_angular_momentum = 0;

	Vector2 rep_velocity_differential = { 0, 0 };
	real_t rep_restitution_coefficient = 0;

	Vector<Color> line_color;
	Vector<real_t> line_width;
	Vector<real_t> line_data;
	Vector<Vector2> line_beginnings;
	Vector<Vector2> line_endings;

	Vector<Line2D *> lines;
	Vector<Label *> line_labels;

	int visdat_objects_generated = 0;
	int visdat_objects_used = 0;

	void collision_visualizer_clear_data();
	void collision_visualizer_add_line(Color _color, real_t _width, String _data, Vector2 _beginning, Vector2 _end);
	void collision_visualizer_generate_data();
#endif

#ifdef POLYPHYSICS_SHOULD_STEP_ON_COLLISION
	bool HALT = false;
#endif

	real_t get_solver_physics_time();
	real_t get_solver_linear_resolution();
	real_t get_solver_angular_resolution();

	/// <returns>Position of insertion (needed for removal function).</returns>
	void add_entity(PhysicsEntity2D *entity);
	void remove_entity(int entity_id); // We will rely on recipients to remove themselves as needed

	/// <returns>the collision pair's ID</returns>
	void add_collision_pair(size_t body1, size_t body2);
	bool collision_pair_already_has_data(size_t collision_pair_id, size_t body1, size_t body2);
	void remove_collision_pair(size_t collision_pair_id);

	// Used to check if a collision could happen - very cheap
	bool check_collision_broadphase(size_t first_entity_loc, size_t second_entity_loc);
	// Used to check if a collision actually happened - ridicolously expensive
	bool check_collision_narrowphase(size_t collision_pair_loc_not_id);
	// Extension for check_collision_narrowphase(). Result is only changed if parameter_t for any of the calculations is smaller than the input one.
	// Returns true if there was a collision
	bool narrowphase_collision_compute(Vector<Vector<Vector2>> &body1_starting_points_vec, Vector<Vector<Vector2>> &body1_ending_points_vec,
			Vector<Vector<Vector2>> &body2_starting_points_vec, Vector<Vector<Vector2>> &body2_ending_points_vec,
			LinePlaneIntersectResult3D &result, CollisionInfo &info);
	// Extension for narrowphase_collision_compute(). Result is only changed if parameter_t for any of the calculations is smaller than the input one.
	// Returns true if there was a collision
	bool narrowphase_collision_do_compute(Vector2 line1point0, Vector2 line1point1,
			Vector2 plane_point_bottom_first, Vector2 plane_point_bottom_second, Vector2 plane_point_top_first, Vector2 plane_point_top_second,
			LinePlaneIntersectResult3D &result);

	/// <returns>false if there were no real collisions</returns>
	void resolve_collision_pairs(); 
	void clear_collision_data();

	void resolve_collision(int collision_pair_loc);
};
