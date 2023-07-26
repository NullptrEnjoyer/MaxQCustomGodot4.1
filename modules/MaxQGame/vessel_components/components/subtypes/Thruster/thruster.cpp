/* thruster.cpp */

#include "thruster.h"

String Thruster::physicsBodyTypeID = typeid(PhysicsEntity2D).raw_name();

Thruster::Thruster() {
	tag = "bgThruster";
	// If changing, also change the thruster movement controller
	// If you add more things which use the tag, add them in comments just in case
}

void Thruster::find_physics_body() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (physicsBodyTypeID != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		releveant_body = static_cast<PhysicsEntity2D *>(parent);

		precompute_forces();
		return;
	}
}

void Thruster::null_physics_body() {
	releveant_body = nullptr;
}

void Thruster::find_sprite() {
	TypedArray<Node> target_vec = get_children();
	int length = target_vec.size();
	for (size_t i = 0; i < length; i++) {
		Object *o = target_vec[i];
		
		if (o->get_class_name() == "Sprite2D") {
			sprite = reinterpret_cast<Sprite2D*>(o);
			return;
		}
	}
}

void Thruster::apply_thrust() {

	if (apply_thrust_next_tick && releveant_body != nullptr) {
		releveant_body->queue_central_force(precomp_force);
		releveant_body->queue_torque(precomp_torque);
		apply_thrust_next_tick = false;

		sprite->set_visible(true);
		return;
	}
	sprite->set_visible(false);
}

void Thruster::autodetermine_dir() {
	if (autodetermine_direction) {
		// Make sure we override these
		move_front = false;
		move_back = false;
		move_left = false;
		move_right = false;

		Vector2 vector = { 0, 1 }; // Pointing "down"
		vector = vector.rotated(get_global_rotation() - releveant_body->get_global_rotation());

		if (abs(vector.x) > abs(vector.y)) {
			if (vector.x < 0) {
				move_right = true;
			} else {
				move_left = true;
			}
		} else {
			if (vector.y > 0) {
				move_front = true;
			} else {
				move_back = true;
			}
		}
	}
}

void Thruster::precompute_forces() {
	if (releveant_body == nullptr) {
		return;
	}

	precomp_force.x = sin(get_global_rotation() - releveant_body->get_global_rotation()) * power_real;
	precomp_force.y = -cos(get_global_rotation() - releveant_body->get_global_rotation()) * power_real;

	Vector2 offset = { 0, 0 };
	offset.x = get_global_position().x - releveant_body->get_global_position().x;
	offset.y = releveant_body->get_global_position().y - get_global_position().y; // Y is flipped, so to get accurate results we have to flip it back

	precomp_torque = (precomp_force.y * offset.x + precomp_force.x * offset.y);

	autodetermine_dir();
}

void Thruster::_notification(int p_what) {
	Component::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {

		case NOTIFICATION_READY: //third
			set_physics_process(true);
			find_sprite();
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			apply_thrust();
			break;

		case 22000: // NOTIFICATION_POLYPHYSICS_RECOMPUTE
			precompute_forces();
			break;

		case NOTIFICATION_NODE_RECACHE_REQUESTED:
			if (is_ready()) {
				null_physics_body();
				find_physics_body();
			}
			break;

		case NOTIFICATION_PREDELETE:
		case NOTIFICATION_EXIT_TREE:
			null_physics_body();
			break;
	}
}

void Thruster::_bind_methods() {
	ADD_GROUP("Physics", "");
	ClassDB::bind_method(D_METHOD("set_power", "power"), &Thruster::set_power);
	ClassDB::bind_method(D_METHOD("get_power"), &Thruster::get_power);
	ClassDB::add_property("Thruster", PropertyInfo(Variant::FLOAT, "power_(kN)"), "set_power", "get_power");

	ADD_GROUP("Logic", "");
	ClassDB::bind_method(D_METHOD("set_main_engine", "bool"), &Thruster::set_main_engine);
	ClassDB::bind_method(D_METHOD("get_main_engine"), &Thruster::get_main_engine);
	ClassDB::add_property("Thruster", PropertyInfo(Variant::BOOL, "is_main_engine"), "set_main_engine", "get_main_engine");

	ADD_GROUP("Rotation Logic", "");

	ClassDB::bind_method(D_METHOD("set_rotate_left", "bool"), &Thruster::set_rotate_left);
	ClassDB::bind_method(D_METHOD("get_rotate_left"), &Thruster::get_rotate_left);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "rotate_left"), "set_rotate_left", "get_rotate_left"); // I could've just done this from the start lol
	ClassDB::bind_method(D_METHOD("set_rotate_right", "bool"), &Thruster::set_rotate_right);
	ClassDB::bind_method(D_METHOD("get_rotate_right"), &Thruster::get_rotate_right);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "rotate_right"), "set_rotate_right", "get_rotate_right");

	ADD_GROUP("Direction Logic Automation", "");
	ClassDB::bind_method(D_METHOD("set_autodetermine_dir", "bool"), &Thruster::set_autodetermine_dir);
	ClassDB::bind_method(D_METHOD("get_autodetermine_dir"), &Thruster::get_autodetermine_dir);
	ClassDB::add_property("Thruster", PropertyInfo(Variant::BOOL, "autodetermine_direction"), "set_autodetermine_dir", "get_autodetermine_dir");

	// Did you know altgr + click makes you type in multiple things at once? I did, and it just saved my life.
	// Thanks for coming to my Ted talk
	ADD_GROUP("Direction Logic", "");
	ClassDB::bind_method(D_METHOD("set_move_front", "bool"), &Thruster::set_move_front);
	ClassDB::bind_method(D_METHOD("get_move_front"), &Thruster::get_move_front);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "move_front"), "set_move_front", "get_move_front");
	ClassDB::bind_method(D_METHOD("set_move_back", "bool"), &Thruster::set_move_back);
	ClassDB::bind_method(D_METHOD("get_move_back"), &Thruster::get_move_back);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "move_back"), "set_move_back", "get_move_back");
	ClassDB::bind_method(D_METHOD("set_move_left", "bool"), &Thruster::set_move_left);
	ClassDB::bind_method(D_METHOD("get_move_left"), &Thruster::get_move_left);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "move_left"), "set_move_left", "get_move_left");
	ClassDB::bind_method(D_METHOD("set_move_right", "bool"), &Thruster::set_move_right);
	ClassDB::bind_method(D_METHOD("get_move_right"), &Thruster::get_move_right);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "move_right"), "set_move_right", "get_move_right");
}
