/* component.cpp */

#include "thruster_movement_controller.h"

ThrusterMovementController::ThrusterMovementController() {
	tag = "bgThrustController";
	target_id = ComponentIDManager::get_singleton()->fetch_id("bgThruster");
}

void ThrusterMovementController::_notification(int p_what) {
	Component::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_READY:
			set_physics_process(true);
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			move();
			break;
	}
}

void ThrusterMovementController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("move_fullspeed"), &ThrusterMovementController::move_fullspeed);
	ClassDB::bind_method(D_METHOD("move_front"), &ThrusterMovementController::move_front);
	ClassDB::bind_method(D_METHOD("move_back"), &ThrusterMovementController::move_back);
	ClassDB::bind_method(D_METHOD("move_left"), &ThrusterMovementController::move_left);
	ClassDB::bind_method(D_METHOD("move_right"), &ThrusterMovementController::move_right);
	ClassDB::bind_method(D_METHOD("rotate_left"), &ThrusterMovementController::rotate_left);
	ClassDB::bind_method(D_METHOD("rotate_right"), &ThrusterMovementController::rotate_right);
}

void ThrusterMovementController::reset_vars() {
	full = false;
	front = false;
	back = false;
	left = false;
	right = false;
	rot_left = false;
	rot_right = false;
}

void ThrusterMovementController::move() {
	const Vector<Thruster *> *target_vec = reinterpret_cast<const Vector<Thruster *> *>(curr_sorter->get_component_vec(target_id));
	if (target_vec != nullptr) {
		int length = target_vec->size();

		for (size_t i = 0; i < length; i++) {
			Thruster *target_thruster = target_vec->get(i);

			if (!full && target_thruster->main_engine) {
				continue;
			}

			if ((front && target_thruster->move_front) || (back && target_thruster->move_back) || (left && target_thruster->move_left) ||
				(right && target_thruster->move_right) || (rot_left && target_thruster->rotate_left) || (rot_right && target_thruster->rotate_right)) {
				target_thruster->apply_thrust_next_tick = true;
			}
		}
	}
	reset_vars();
}

void ThrusterMovementController::move_fullspeed() {
	full = true;
}

void ThrusterMovementController::move_front() {
	front = true;
}

void ThrusterMovementController::move_back() {
	back = true;
}

void ThrusterMovementController::move_left() {
	left = true;
}

void ThrusterMovementController::move_right() {
	right = true;
}

void ThrusterMovementController::rotate_left() {
	rot_left = true;
}

void ThrusterMovementController::rotate_right() {
	rot_right = true;
}
