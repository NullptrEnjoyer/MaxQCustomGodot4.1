/* independent_physics_solver_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../physics_entity/physics_entity_2d.h"
#include "../body_segment/body_segment_2d.h"

class PhysicsEntity2D; //<---- CIRCULAR DEPENDENCY
class PhysicsSegment2D; //<---- CIRCULAR DEPENDENCY

struct CollisionPair {
	size_t ID = 0;
	real_t initial_toi = 0;
	real_t toi = 0;

	// The indexes for the affected bodies within our entities vector.
	// Segments are also indexes, used in resolution as they can have very different properties
	size_t body1 = 0;
	size_t body1_affected_segment = 0;
	size_t body2 = 0;
	size_t body2_affected_segment = 0;

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

//	static void _bind_methods();
	void _notification(int p_what);
	void tick();

	Vector<PhysicsEntity2D *> entity_vec;
	Vector<CollisionPair> collision_pair_vec;
	Vector<size_t> collision_pair_id_vec;

public:

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
	// Extension for check_collision_narrowphase(). result is only changed if parameter_t for any of the calculations is smaller than the input one.
	// Returns true if there was a collision
	bool narrowphase_collision_compute(Vector<Vector<Vector2>> &body1_starting_points_vec, Vector<Vector<Vector2>> &body1_ending_points_vec,
			Vector<Vector<Vector2>> &body2_starting_points_vec, Vector<Vector<Vector2>> &body2_ending_points_vec, LinePlaneIntersectResult3D& result
		);

	/// <returns>false if there were no real collisions</returns>
	void resolve_collision_pairs(); 
	void clear_collision_data();
};
