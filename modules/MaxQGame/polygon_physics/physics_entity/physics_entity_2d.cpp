/* physics_entity_2d.cpp */

//TODO: calculate total center of mass
//TODO: calculate inertia
//TODO: rotation relative to center of mass

#include "physics_entity_2d.h"
#include <limits>

String PhysicsEntity2D::PhysicsSolverTypeID = typeid(ThreadedPhysicsSolver2D).raw_name();

void PhysicsEntity2D::try_find_physics_controller() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (PhysicsSolverTypeID != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		solver = static_cast<ThreadedPhysicsSolver2D *>(parent);
		ID = solver->add_entity(this);
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

void PhysicsEntity2D::apply_queued_forces() {
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

void PhysicsEntity2D::force_to_velocity(Vector2 force) {
	velocity += force / total_mass;
}

void PhysicsEntity2D::torque_to_angular_velocity(real_t torque) {
	angular_velocity += torque / total_inertia;
}

void PhysicsEntity2D::step() {
	if (calculate_data_from_segments_next_step) {
		calculate_data_from_segments();
	}
	set_global_position(get_global_position() + velocity * get_physics_process_delta_time());
	set_global_rotation(get_global_rotation() + angular_velocity * get_physics_process_delta_time());
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

	// To keep things in the same position we can just move all our segments for -center_of_mass
	{
		Vector2 move_by = -center_of_mass;

		for (size_t i = 0; i < physics_segments_len; i++) {
			physics_segments[i]->set_position(physics_segments[i]->get_position() + move_by);
		}
	}

	// Now we find our inertia. Get it for each individual segment and add them up

	// We can also do our inertia calculations here as we already have the total mass
	total_inertia = 0;
	{
		for (int i = 0; i < physics_segments_len; i++) {
			// This is how inertia is done in godot, probably for the best as other ways get very expensive very quickly
			Vector<Vector2> polygon_vec = physics_segments[i]->get_polygon();
			int vec_size = polygon_vec.size();

			Rect2 aabb_new;
			aabb_new.position = polygon_vec[0];
			for (int i = 0; i < vec_size; i++) {
				aabb_new.expand_to(polygon_vec[i]);
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
