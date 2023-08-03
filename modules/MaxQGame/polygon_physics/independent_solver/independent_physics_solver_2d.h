/* independent_physics_solver_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../physics_entity/physics_entity_2d.h"
#include "../body_segment/body_segment_2d.h"

class PhysicsEntity2D; //<---- CIRCULAR DEPENDENCY
class PhysicsSegment2D; //<---- CIRCULAR DEPENDENCY?

struct CollisionPair {
	size_t ID = 0;
	real_t initial_toi = 0;
	real_t toi = 0;
	// The indexes for the affected bodies within our entities vector
	size_t body1 = 0;
	size_t body2 = 0;
	Vector2 impact_point = {0, 0};
	// These 4 contain data from all segments within an entity
	Vector<Vector2> b1_polygon_vec;
	Vector<Vector2> b1_future_polygon_vec;
	Vector<Vector2> b2_polygon_vec;
	Vector<Vector2> b2_future_polygon_vec;
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

	void clear_collision_data();
};
