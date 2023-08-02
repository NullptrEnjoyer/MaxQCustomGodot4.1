/* physics_entity_2d.cpp */

#include "physics_entity_2d.h"
//#include <limits> // If something breaks reenable this?

String PhysicsEntity2D::PhysicsSolverTypeID = typeid(IndependentPhysicsSolver2D).raw_name();

void PhysicsEntity2D::try_find_physics_controller() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (PhysicsSolverTypeID != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		solver = static_cast<IndependentPhysicsSolver2D *>(parent);
		solver->add_entity(this); // Also assigns us an id via set_id
		return;
	}
}

void PhysicsEntity2D::try_remove_from_physics_controller() {
	if (solver != nullptr) {
		solver->remove_entity(ID);
		solver = nullptr;
	}
}

void PhysicsEntity2D::queue_central_force(Vector2 force) {
	queued_force += force;
}

void PhysicsEntity2D::queue_torque(real_t torque) {
	queued_torque += torque;
}

void PhysicsEntity2D::force_to_velocity(Vector2 force) {
	velocity += force / total_mass;
}

void PhysicsEntity2D::torque_to_angular_velocity(real_t torque) {
	angular_velocity += torque / total_inertia;
}

Vector<PhysicsSegment2D *> PhysicsEntity2D::get_segments() {
	return physics_segments;
}

int PhysicsEntity2D::add_segment(PhysicsSegment2D *physics_segment) {
	int pos = physics_segments.size();
	physics_segments.push_back(physics_segment);
	queue_calculate_data_from_segments();
	return pos;
}

void PhysicsEntity2D::remove_segment_at(int id) {
	physics_segments.remove_at(id);
	queue_calculate_data_from_segments();
}

void PhysicsEntity2D::queue_calculate_data_from_segments() {
	calculate_data_from_segments_next_step = true;
}

void PhysicsEntity2D::calculate_data_from_segments() {
	calculate_data_from_segments_next_step = false;// I fogor lol

	int physics_segments_len = physics_segments.size();
	real_t mass_to_check = 0;
	total_mass = 0;

	if (physics_segments_len <= 0) { // Bruh
		return;
	}

	// https://www.khanacademy.org/science/physics/linear-momentum/center-of-mass/v/center-of-mass-equation
	// We find the center of mass from all the segments in our ship
	Vector2 center_of_mass = { 0, 0 };
	{
		for (int i = 0; i < physics_segments_len; i++) {
			mass_to_check = physics_segments[i]->get_mass();
			total_mass += mass_to_check;
			center_of_mass += physics_segments[i]->get_position() * mass_to_check;
		}
		center_of_mass /= total_mass;
		set_position(get_position() + center_of_mass);
	}

	// To keep everything in the correct position we can just move all our segments for -center_of_mass
	// This is also fairly light, so we setup our bounds rect here.
	{
		Vector2 move_by = -center_of_mass;

		if (physics_segments[0]->get_polygon().size() > 0) {
			local_bounding_box.set_position(physics_segments[0]->get_position() + physics_segments[0]->get_polygon()[0]);
		} else {
			local_bounding_box = { 0, 0, 0, 0 };
		}

		for (size_t i = 0; i < physics_segments_len; i++) {
			physics_segments[i]->set_position(physics_segments[i]->get_position() + move_by);

			// Bounding box calculation, likely very inefficient
			const Vector<Vector2> polygon_vec_const = physics_segments[i]->get_polygon();
			Vector2 polygon_pos = physics_segments[i]->get_position();
			size_t physics_segment_point_num = polygon_vec_const.size();
			for (size_t j = 0; j < physics_segment_point_num; j++) {
				local_bounding_box.expand_to(polygon_vec_const[j] + polygon_pos);
			}
		}
		// Increase it by 2 * so we also account for rotation
		// Yes it's approximate, shut
		local_bounding_box.position -= local_bounding_box.size / 2;
		local_bounding_box.size *= 2;
	}

	// We can also do our inertia calculations here as we already have the total mass
	{
		total_inertia = 0;

		for (int i = 0; i < physics_segments_len; i++) {
			// This is how inertia is done in godot, probably for the best as other ways get very expensive very quickly
			Vector<Vector2> polygon_vec = physics_segments[i]->get_polygon();
			int vec_size = polygon_vec.size();

			Rect2 aabb_new;
			aabb_new.position = polygon_vec[0];
			for (int j = 0; j < vec_size; j++) {
				aabb_new.expand_to(polygon_vec[j]);
			}
			total_inertia += total_mass * aabb_new.size.dot(aabb_new.size) / 12.0;
		}
	}

	propagate_notification(22000); // NOTIFICATION_POLYPHYSICS_RECOMPUTE
}

void PhysicsEntity2D::_notification(int p_what) {
	Node2D::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_READY: {
			try_find_physics_controller();
		}break;

		case NOTIFICATION_NODE_RECACHE_REQUESTED: {
			if (is_ready()) {
				try_remove_from_physics_controller();
				try_find_physics_controller();
			}
		} break;

		case NOTIFICATION_PREDELETE:
		case NOTIFICATION_EXIT_TREE:
			try_remove_from_physics_controller();
			break;
	}
}

// Solver and collision resolution stuff

void PhysicsEntity2D::set_id(size_t index) {
	ID = index;
}

void PhysicsEntity2D::solver_apply_queued_forces() {
	if (queued_force.x != 0 || queued_force.y != 0) {
		queued_force = queued_force.rotated(get_global_rotation());

		queued_force *= get_physics_process_delta_time();

		force_to_velocity(queued_force);
		queued_force = { 0, 0 };
	}

	if (queued_torque != 0) {
		queued_torque *= get_physics_process_delta_time();

		torque_to_angular_velocity(queued_torque);
		queued_torque = 0;
	}
}

void PhysicsEntity2D::solver_prepare_for_step() {
	Vector2 position_delta = velocity * (get_physics_process_delta_time() - total_toi);

	real_bounding_box = local_bounding_box;
	real_bounding_box.position += get_global_position();
	future_bounding_box = real_bounding_box;
	future_bounding_box.position += position_delta;

	collision_bounding_box.position = real_bounding_box.position;
	collision_bounding_box.expand_to(collision_bounding_box.position + collision_bounding_box.size);
	collision_bounding_box.expand_to(future_bounding_box.position);
	collision_bounding_box.expand_to(future_bounding_box.position + future_bounding_box.size);
}

bool PhysicsEntity2D::solver_check_collision_broadphase(PhysicsEntity2D *other) {
	if (collision_bounding_box.intersects(other->collision_bounding_box)) {
		return true;
	}
	return false;
}

void PhysicsEntity2D::solver_step_for_toi(real_t toi) {
	total_toi += toi;
	set_global_position(get_global_position() + velocity * toi);
	set_global_rotation(get_global_rotation() + angular_velocity * toi);
}

void PhysicsEntity2D::solver_step() {
	if (calculate_data_from_segments_next_step) {
		calculate_data_from_segments();
	}

	real_t time = (get_physics_process_delta_time() - total_toi);
	set_global_position(get_global_position() + velocity * time);
	set_global_rotation(get_global_rotation() + angular_velocity * time);
	clear_collision_data();
}

void PhysicsEntity2D::clear_collision_data() {
	total_toi = 0;
}

real_t PhysicsEntity2D::get_total_toi() {
	return total_toi;
}

void PhysicsEntity2D::add_collision_pair(real_t pair_id) {
	connected_collision_pair_id_vec.push_back(pair_id);
}

bool PhysicsEntity2D::check_if_collision_pair_exists(real_t other_body_id) {
	size_t length = connected_collision_pair_id_vec.size();

	for (size_t i = 0; i < length; i++) {
		if (solver->collision_pair_already_has_data(connected_collision_pair_id_vec[i], this->ID, other_body_id)) {
			return true;
		}
	}

	return false;
}

void PhysicsEntity2D::remove_collision_pair(real_t pair_id) {
	int length = connected_collision_pair_id_vec.size();

	for (int i = length; i > 0; i--) {
		if (connected_collision_pair_id_vec[i] == pair_id) {
			connected_collision_pair_id_vec.remove_at(i);
		}
	}
}

void PhysicsEntity2D::clear_collision_pair_vec() {
	int length = connected_collision_pair_id_vec.size();

	for (int i = length; i > 0; i--) {
		solver->remove_collision_pair(connected_collision_pair_id_vec[i]);
	}
}
