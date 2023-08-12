/* physics_entity_2d.cpp */

#include "physics_entity_2d.h"
#include "../poly_physics_2d_defines.h"
//#include <limits> // If something breaks reenable this?

//Physics controller stuff
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
		solver->remove_entity(physics_id);
		solver = nullptr;
	}
}

Rect2 PhysicsEntity2D::get_collision_box() {
	return collision_bounding_box;
}

Vector2 PhysicsEntity2D::get_velocity() {
	return velocity;
}

real_t PhysicsEntity2D::get_angular_velocity() {
	return angular_velocity;
}

// Force calculations
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

	if (physics_segments_len <= 0) { // Bruh
		return;
	}

	real_t mass_to_check = 0;
	total_mass = 0;

	// https://www.khanacademy.org/science/physics/linear-momentum/center-of-mass/v/center-of-mass-equation
	// We find the center of mass from all the segments in our ship
	Vector2 global_center_of_mass = { 0, 0 };
	Vector2 segment_center_of_mass = { 0, 0 };
	Vector2 position_change_amount= { 0, 0 };
	{
		// We must account for rotation and all that
		for (int i = 0; i < physics_segments_len; i++) {
			mass_to_check = physics_segments[i]->get_mass();
			segment_center_of_mass = physics_segments[i]->get_center_of_mass().rotated(physics_segments[i]->get_global_rotation()) + physics_segments[i]->get_global_position();
			total_mass += mass_to_check;
			global_center_of_mass += ((segment_center_of_mass) * mass_to_check);
		}
		global_center_of_mass /= total_mass;
		position_change_amount = get_global_position() - global_center_of_mass;
		set_global_position(global_center_of_mass);
	}

	// To keep everything in the correct position we can just move all our segments for -center_of_mass
	{
		for (size_t i = 0; i < physics_segments_len; i++) {
			physics_segments[i]->set_global_position(physics_segments[i]->get_global_position() + position_change_amount);
		}
	}

	// Bounding box calculation. No need to rotate everything, we expland the box by 2 times at the end to account for rotation anyway
	{
		if (physics_segments[0]->get_polygon().size() > 0 && physics_segments[0]->get_polygon().size() > 0) {
			local_bounding_box.set_position(physics_segments[0]->get_polygon()[0]);
		} else {
			local_bounding_box = { 0, 0, 0, 0 };
		}

		for (size_t i = 0; i < physics_segments_len; i++) {
			const Vector<Vector2> polygon_vec_const = physics_segments[i]->get_polygon();
			size_t physics_segment_point_num = polygon_vec_const.size();
			for (size_t j = 0; j < physics_segment_point_num; j++) {
				local_bounding_box.expand_to(polygon_vec_const[j]);
			}
		}

		// Should make it a box. Cuz uh. Bounding box. I mean come on.
		if (local_bounding_box.size.x > local_bounding_box.size.y) {
			real_t delta = local_bounding_box.size.x - local_bounding_box.size.y;
			local_bounding_box.size.y += delta;
			local_bounding_box.position.x -= delta / 2;
		} else {
			real_t delta = local_bounding_box.size.y - local_bounding_box.size.x;
			local_bounding_box.size.x += delta;
			local_bounding_box.position.y -= delta / 2;
		}

		local_bounding_box.size *= 1.5;

		local_bounding_box.position = { 0, 0 };
		local_bounding_box.position = -local_bounding_box.size * 0.5;

		// Increase it by 1.5 * so we also account for possible rotation
		// Yes it's approximate, shut
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

	// I set things like precomputed forces up so things are a lot faster, but the price we pay is having to update them whenever we change stuff
	// Thankfully, godot has this neat notification system which lets us notify everything below us about our predicament
	propagate_notification(POLYPHYSICS_NOTIFICATION_RECOMPUTE);
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

void PhysicsEntity2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_velocity"), &PhysicsEntity2D::get_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &PhysicsEntity2D::get_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_bounding_rect"), &PhysicsEntity2D::get_collision_box);
}

// Solver and collision resolution stuff. Only the physics solver should call these
void PhysicsEntity2D::set_id(size_t index) {
	physics_id = index;
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

	collision_bounding_box = real_bounding_box;
	collision_bounding_box.expand_to(future_bounding_box.position);
	collision_bounding_box.expand_to(future_bounding_box.get_end());
}

void PhysicsEntity2D::solver_step_for_toi(real_t toi) {
	total_toi += toi;
	set_global_position(get_global_position() + velocity * toi);
	set_global_rotation(get_global_rotation() + angular_velocity * toi);
	clear_segment_position_data();
	solver_prepare_for_step(); // We must regenerate our bounding box
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
	clear_segment_position_data();
}

void PhysicsEntity2D::clear_segment_position_data() {
	Vector<PhysicsSegment2D *> segments = get_segments();
	size_t length = segments.size();

	for (size_t i = 0; i < length; i++) {
		segments[i]->entity_clear_position_data();
	}
}

real_t PhysicsEntity2D::get_total_toi() {
	return total_toi;
}

void PhysicsEntity2D::solver_add_collision_pair(size_t pair_id) {
	connected_collision_pair_id_vec.push_back(pair_id);
}

bool PhysicsEntity2D::solver_collision_pair_already_exists(size_t other_body_id) {
	size_t length = connected_collision_pair_id_vec.size();
	for (size_t i = 0; i < length; i++) {
		if (solver->collision_pair_already_has_data(connected_collision_pair_id_vec[i], this->physics_id, other_body_id)) {
			return true;
		}
	}
	return false;
}

void PhysicsEntity2D::solver_remove_collision_pair(size_t pair_id) {
	int length = connected_collision_pair_id_vec.size();

	for (int i = length - 1; i > 0; i--) {
		if (connected_collision_pair_id_vec[i] == pair_id) {
			connected_collision_pair_id_vec.remove_at(i);
		}
	}
}

void PhysicsEntity2D::solver_invalidate_collision_pair_vec() {
	int length = connected_collision_pair_id_vec.size();
	for (int i = length - 1; i >= 0; i--) {
		solver->remove_collision_pair(connected_collision_pair_id_vec[i]);
	}
}

void PhysicsEntity2D::solver_clear_collision_pair_vec() {
	connected_collision_pair_id_vec.clear();
}

void PhysicsEntity2D::load_all_current_points(Vector<Vector<Vector2>> &target_vec) {
	size_t length = physics_segments.size();
	for (size_t i = 0; i < length; i++) {
		target_vec.push_back(Vector<Vector2>(physics_segments[i]->entity_get_global_currpos()));
	}
}

void PhysicsEntity2D::load_all_specified_timestep_points(Vector<Vector<Vector2>> &target_vec, real_t target_time) {
	size_t length = physics_segments.size();
	for (size_t i = 0; i < length; i++) {
		target_vec.push_back(Vector<Vector2>(physics_segments[i]->entity_get_global_thispos(target_time)));
	}
}

void PhysicsEntity2D::load_all_final_points(Vector<Vector<Vector2>> &target_vec) {
	size_t length = physics_segments.size();
	for (size_t i = 0; i < length; i++) {
		target_vec.push_back(Vector<Vector2>(physics_segments[i]->entity_get_global_nextpos()));
	}
}
/*
bool PhysicsEntity2D::solver_find_outbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, PhysicsEntity2D *other) {
	size_t segments_length = physics_segments.size();
	bool returnbool = false;

	for (size_t i = 0; i < segments_length; i++) {
		Vector<Vector2> currpos = physics_segments[i]->entity_get_global_currpos();
		Vector<Vector2> nextpos = physics_segments[i]->entity_get_global_nextpos();

		size_t points_length = physics_segments.size();
		LinePlaneIntersectResult3D result_out;
		real_t toi_out = 0;
		for (size_t j = 0; j < points_length; j++) {
			if ((other->solver_find_inbound_collision_with_smallest_toi(result_out, toi_out, currpos[j], nextpos[j], total_toi))
					&& (result_out.param_t >= result.param_t)) {
				returnbool = true;
				result.equalize(result_out);
				time_of_impact = toi_out;
			}
		}
	}

	return returnbool;
}

bool PhysicsEntity2D::solver_find_inbound_collision_with_smallest_toi(LinePlaneIntersectResult3D &result, real_t &time_of_impact, Vector2 global_start_point,
																		Vector2 global_end_point, real_t starting_deltatime) {
	Vector2 real_start_point = global_start_point;
	Vector<Vector2> current_segment_points_vec;
	Vector<Vector2> next_segment_points_vec;
	Vector<PhysicsSegment2D *> segments_vec = get_segments();
	size_t num_of_segments = segments_vec.size();
	real_t for_time = 0;

	if (starting_deltatime == total_toi) {// We're synced up and good to go
		for_time = get_physics_process_delta_time() - starting_deltatime;
		for (size_t i = 0; i < num_of_segments; i++) {
			current_segment_points_vec.append_array(segments_vec[i]->entity_get_global_currpos());
		}
	} else if (starting_deltatime > total_toi) {// We are behind in time, let's move up.
		for_time = get_physics_process_delta_time() - starting_deltatime;
		for (size_t i = 0; i < num_of_segments; i++) {
			current_segment_points_vec.append_array(segments_vec[i]->entity_get_global_thispos(starting_deltatime));
		}
	} else {// The inbound points are behind in time, we must correct the starting one.
		for_time = get_physics_process_delta_time() - total_toi;
		real_t multip_factor = (total_toi - starting_deltatime) / (get_physics_process_delta_time() - starting_deltatime);
		real_start_point = (global_end_point - global_start_point) * multip_factor;

		for (size_t i = 0; i < num_of_segments; i++) {
			current_segment_points_vec.append_array(segments_vec[i]->entity_get_global_currpos());
		}
	}

	for (size_t i = 0; i < num_of_segments; i++) {
		next_segment_points_vec.append_array(segments_vec[i]->entity_get_global_nextpos());
	}
}*/
