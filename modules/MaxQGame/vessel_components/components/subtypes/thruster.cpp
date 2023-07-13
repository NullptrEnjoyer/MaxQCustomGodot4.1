/* thruster.cpp */

#include "thruster.h"

String Thruster::physicsBodyTypeID = typeid(RigidBody2D).raw_name();

Thruster::Thruster() {
	tag = "bgThruster";
}

void Thruster::find_physics_body() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (physicsBodyTypeID != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		releveant_body = static_cast<RigidBody2D *>(parent);

		precompute_forces();
		autodetermine_dir();
		return;
	}
}

void Thruster::null_physics_body() {
	releveant_body = nullptr;
}

void Thruster::apply_thrust() {
	if (apply_thrust_next_tick && releveant_body != nullptr) {
		releveant_body->apply_central_force(precomp_force);
		releveant_body->apply_torque(precomp_torque);
		apply_thrust_next_tick = false;
	}
}

void Thruster::autodetermine_dir() {
}

void Thruster::precompute_forces() {
	precomp_force.x = sin(get_rotation()) * power;
	precomp_force.y = -cos(get_rotation()) * power;

	Vector2 offset = { 0, 0 };
	offset.x = get_position().x - releveant_body->get_center_of_mass().x;
	offset.y = get_position().y - releveant_body->get_center_of_mass().y;

	precomp_torque = -(offset.x * precomp_force.y + offset.y * precomp_force.x);
}

void Thruster::_notification(int p_what) {
	Component::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_PARENTED: //first
			if (!ready) {
				ready = true;
				power *= 20000000; // UNNNNNNNNNNNNNLLLIMITEEEEEEEDDDDDDDDDDDDDD POOOOOOOOOOOOOOOOWWWERRRRRRRRRRRRRRRRRRRRRRRR
			}
			find_physics_body();
			break;

		case NOTIFICATION_READY: //third
			set_physics_process(true);
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			apply_thrust();
			break;

		case NOTIFICATION_PREDELETE:
		case NOTIFICATION_UNPARENTED:
		case NOTIFICATION_EXIT_TREE:
			null_physics_body();
			break;
	}
}

void Thruster::_bind_methods() {
	//PropertyInfo(Variant::BOOL, "amount", PROPERTY_HINT_RANGE, "0,49,1", PROPERTY_USAGE_EDITOR);
}
