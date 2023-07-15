/* custom_physics_body_2d.cpp */

#include "custom_physics_body_2d.h"

void MaxQRigidBody2D::_notification(int p_what) {
	//RigidBody2D::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_READY: //third
			set_physics_process(true);
			break;
		case NOTIFICATION_PHYSICS_PROCESS: //third
			if (queued_force.x != 0 || queued_force.y != 0) {
				queued_force = queued_force.rotated(get_global_rotation());
				queued_force *= get_physics_process_delta_time();

				apply_central_force(queued_force);
				queued_force = { 0, 0 };
			}
			if (queued_torque != 0) {
				queued_torque *= get_physics_process_delta_time();

				apply_torque(queued_torque);
				queued_torque = 0;
			}
			break;
	}
}

void MaxQRigidBody2D::queue_central_force(Vector2 force) {
	queued_force += force;
}

void MaxQRigidBody2D::queue_torque(real_t torque) {
	queued_torque += torque;
}
