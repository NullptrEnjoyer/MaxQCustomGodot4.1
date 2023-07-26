/* threaded_physics_solver_2d.h */

#pragma once

#include "../poly_physics_2d.h"
#include "../physics_entity/physics_entity_2d.h"

class PhysicsEntity2D; //<---- CIRCULAR DEPENDENCY

class ThreadedPhysicsSolver2D : public PolygonPhysicsSystem2D {
	GDCLASS(ThreadedPhysicsSolver2D, PolygonPhysicsSystem2D);

protected:

//	static void _bind_methods();
	void _notification(int p_what);
	void tick();

	Vector<PhysicsEntity2D *> entities;

public:
	/// <returns>Position of insertion (needed for removal function).</returns>
	int add_entity(PhysicsEntity2D *detector);
	void remove_entity(int detector_id); // We will rely on recipients to remove themselves as needed

};
