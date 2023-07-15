/* custom_physics_body_2d.h */

#pragma once

#include "scene/2d/physics_body_2d.h"

class MaxQRigidBody2D : public RigidBody2D {
	GDCLASS(MaxQRigidBody2D, RigidBody2D);

protected:
	//void _notification(int p_what);

	Vector2 queued_force{0, 0};
	real_t queued_torque = 0;

	void _notification(int p_what);

public:
	void queue_central_force(Vector2 force);
	void queue_torque(real_t torque);
};
