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

void PhysicsEntity2D::set_total_mass(real_t mass) {
	total_mass = mass;
	inverse_total_mass = 1 / mass;
}

void PhysicsEntity2D::set_total_inertia(real_t inertia) {
	total_inertia = inertia;
	inverse_total_inertia = 1 / inertia;
}

Rect2 PhysicsEntity2D::get_collision_box() {
	return collision_bounding_box;
}

real_t PhysicsEntity2D::get_total_mass() {
	return total_mass;
}

real_t PhysicsEntity2D::get_total_inertia() {
	return total_inertia;
}

real_t PhysicsEntity2D::get_inverse_total_mass() {
	return inverse_total_mass;
}

real_t PhysicsEntity2D::get_inverse_total_inertia() {
	return inverse_total_inertia;
}

Vector2 PhysicsEntity2D::get_velocity() {
	return velocity;
}

real_t PhysicsEntity2D::get_angular_velocity() {
	return angular_velocity;
}

/*
int PhysicsEntity2D::which_segment_contains_point(Vector2 global_point) {
	So this is actually quite simple! We are going to raycast to the left and count how many times we hit the border.
	If the number is even, we are outside the polygon. If it it odd, we are inside!
	This is a nice method which doesn't even need the polygon to be convex.

	// IN CASE OF A MAJOR REFACTOR, BREAK GLASS
	GLASS_START

	VectorVectorIntersectResult2D res;
	return get_point_displacement_vec(global_point, { 1, 0 }, &res);

	GLASS_END

	Vector<PhysicsSegment2D *> segments_ptr_vec = get_segments();
	size_t segments_len = segments_ptr_vec.size();

	// I lied, going right is tight
	Vector2 dir_vector = {1,0};

	for (size_t i = 0; i < segments_len; i++) {

		int hits = 0;
		Vector<Vector2> segment_points = segments_ptr_vec[i]->entity_get_global_currpos();
		size_t segment_points_len = segment_points.size();

		VectorVectorIntersectResult2D result;

		if (unlikely(segment_points_len < 2)) {
			print_error("Unitended behaviour @ PhysicsEntity2D::which_segment_contains_point, there is less than two points in a segment!");
			continue;
		}

		// Let's go last to first first!
		solve_vector_vector_collision_with_result(global_point, dir_vector, segment_points[segment_points_len - 1],
				segment_points[0] - segment_points[segment_points_len - 1], &result);
		if (result.get_param_t() >= 0 && result.get_param_u() > 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result.get_param_u() < 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
			++hits;
		}

		if (unlikely(segment_points_len < 3)) {
			print_error("Unitended behaviour @ PhysicsEntity2D::which_segment_contains_point, there is less than three points in a segment!");
			continue;
		}
		
		for (size_t j = 1; j < segment_points_len; j++) {
			// Now we do the rest
			solve_vector_vector_collision_with_result(global_point, dir_vector, segment_points[j - 1],
					segment_points[j] - segment_points[j - 1], &result);
			if (result.get_param_t() >= 0 && result.get_param_u() > 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result.get_param_u() < 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
				++hits;
			}
		}

		if (hits % 2 != 0) {
			return i;
		}
	}

	return -1;
}

int PhysicsEntity2D::get_point_displacement_vec(Vector2 global_point, Vector2 direction_vec, VectorVectorIntersectResult2D *result) {

	if (unlikely(result == nullptr)) {
		print_error("Result missing @ PhysicsEntity2D::get_point_displacement_vec");
		return -1;
	}

	Vector<PhysicsSegment2D *> segments_ptr_vec = get_segments();
	size_t segments_len = segments_ptr_vec.size();
	int returnval = -1;

	for (size_t i = 0; i < segments_len; i++) {
		int hits = 0;
		Vector<Vector2> segment_points = segments_ptr_vec[i]->entity_get_global_currpos();
		size_t segment_points_len = segment_points.size();

		VectorVectorIntersectResult2D temp_result;
		VectorVectorIntersectResult2D intermediate_result;

		if (unlikely(segment_points_len < 2)) {
			print_error("Unitended behaviour @ PhysicsEntity2D::which_segment_contains_point, there is less than two points in a segment!");
			continue;
		}

		// Let's go last to first first!
		solve_vector_vector_collision_with_result(global_point, direction_vec, segment_points[segment_points_len - 1],
				segment_points[0] - segment_points[segment_points_len - 1], &temp_result);
		if (temp_result.get_param_t() >= 0 && temp_result.get_param_u() > 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && temp_result.get_param_u() < 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
			if (temp_result.get_param_t() > intermediate_result.get_param_t()) {
				intermediate_result.equalize(&temp_result);
			}
			++hits;
		}

		if (unlikely(segment_points_len < 3)) {
			print_error("Unitended behaviour @ PhysicsEntity2D::which_segment_contains_point, there is less than three points in a segment!");
			continue;
		}

		for (size_t j = 1; j < segment_points_len; j++) {
			// Now we do the rest
			solve_vector_vector_collision_with_result(global_point, direction_vec, segment_points[j - 1],
					segment_points[j] - segment_points[j - 1], &temp_result);
			if (temp_result.get_param_t() >= 0 && temp_result.get_param_u() > 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && temp_result.get_param_u() < 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
				if (temp_result.get_param_t() > intermediate_result.get_param_t()) {
					intermediate_result.equalize(&temp_result);
				}
				++hits;
			}
		}

		if (hits % 2 != 0) {
			if (intermediate_result.get_param_t() > result->get_param_t()) {
				returnval = i;
				result->equalize(&intermediate_result);
			}
		}
	}

	return returnval;
}
*/

// Force calculations
void PhysicsEntity2D::queue_central_force(Vector2 force) {
	queued_force += force;
}

void PhysicsEntity2D::queue_torque(real_t torque) {
	queued_torque += torque;
}

real_t PhysicsEntity2D::get_torque_from_force(Vector2 global_location, Vector2 force) {
	Vector2 offset = global_location - get_global_position();
	return offset.cross(force);
}

void PhysicsEntity2D::apply_force(Vector2 global_location, Vector2 force) {
	if (unlikely(solver == nullptr)) {
		print_error("You're trying to apply force before a physics solver is linked @ PhysicsEntity2D::apply_force");

		apply_central_impulse_force(force);
		apply_impulse_torque(get_torque_from_force(global_location, force));
		return;
	}

	force *= solver->get_solver_physics_time();
	apply_central_impulse_force(force);
	apply_impulse_torque(get_torque_from_force(global_location, force));
}

void PhysicsEntity2D::apply_central_force(Vector2 force) {
	if (unlikely(solver == nullptr)) {
		print_error("You're trying to apply central force before a physics solver is linked @ PhysicsEntity2D::apply_central_force");

		apply_central_impulse_force(force);
		return;
	}

	apply_central_impulse_force(force * solver->get_solver_physics_time());
}

void PhysicsEntity2D::apply_torque(real_t torque) {
	if (unlikely(solver == nullptr)) {
		print_error("You're trying to apply torque before a physics solver is linked @ PhysicsEntity2D::apply_torque");

		apply_impulse_torque(torque);
		return;
	}

	apply_impulse_torque(torque * solver->get_solver_physics_time());
}

void PhysicsEntity2D::apply_impulse_force(Vector2 global_location, Vector2 impulse) {
	apply_central_impulse_force(impulse);
	apply_impulse_torque(get_torque_from_force(global_location, impulse));
}

void PhysicsEntity2D::apply_central_impulse_force(Vector2 impulse) {
	velocity += impulse * get_inverse_total_mass();
}

void PhysicsEntity2D::apply_impulse_torque(real_t impulse_torque) {
	angular_velocity += impulse_torque * get_inverse_total_inertia();
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

	int physics_segments_len = get_segments().size();

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
		set_total_mass(total_mass); // also generates the inverse for future use

		global_center_of_mass *= inverse_total_mass;
		position_change_amount = get_global_position() - global_center_of_mass;
		set_global_position(global_center_of_mass);
	}

	// To keep everything in the correct position we can just move all our segments for -center_of_mass
	{
		for (size_t i = 0; i < physics_segments_len; i++) {
			physics_segments[i]->set_global_position(physics_segments[i]->get_global_position() + position_change_amount);
		}
	}

	//This is where we fill up our polygon position data
	{
		// TODO - WARNING - PERFORMANCE - REUSE INSTEAD OF CALLING CLEAR?
		polygon_data_positions.clear();
		polygon_data_magnitudes.clear();
		polygon_data_angles.clear();
		biggest_magnitude = 0;
		// the order is segment - time - space (points)
		polygon_spacetime_data.clear();

		for (size_t i = 0; i < physics_segments_len; i++) {
			// We can now fill up our polygon data as well. All of this should give us accurate polygon points at rotation of 0

			real_t rotation_differential = physics_segments[i]->get_global_rotation() - get_global_rotation();
			Vector2 position_differential = physics_segments[i]->get_global_position() - get_global_position();
			position_differential = position_differential.rotated(-get_global_rotation());

			Vector<Vector2> polygon = physics_segments[i]->get_polygon();
			size_t polygon_vec_size = polygon.size();

			Vector<Vector2> positions;
			Vector<real_t> magnitudes;
			Vector<real_t> angles;
			// real_t biggest_magnitude = 0; // Should never be negative

			for (size_t j = 0; j < polygon_vec_size; j++) {
				Vector2 resulting_point = (polygon[j]).rotated(rotation_differential) + position_differential;

				//polygon.set(j, resulting_point);
				real_t magnitude = resulting_point.length();
				if (magnitude > biggest_magnitude) {
					biggest_magnitude = magnitude;
				}

				positions.push_back(resulting_point);
				magnitudes.push_back(magnitude);
				angles.push_back(resulting_point.angle());
			}

			polygon_data_positions.push_back(positions);
			polygon_data_magnitudes.push_back(magnitudes);
			polygon_data_angles.push_back(angles);

			//polygon_data.push_back(polygon);
		}

		for (size_t i = 0; i < physics_segments_len; i++) {
			Vector<Vector<Vector3>> time_order_data;
			Vector<Vector3> points_data;
			points_data.resize(polygon_data_positions.ptr()[i].size());
			time_order_data.push_back(points_data);
			polygon_spacetime_data.push_back(time_order_data);
		}
	}

	// And now our bounding box calculation. No need to rotate everything, we expland the box by 2 times at the end to account for all possible rotation anyway
	{
		/*
		real_t biggest_magnitude = 0;

		for (size_t i = 0; i < physics_segments_len; i++) {
			if (biggest_magnitudes.ptr()[i] > biggest_magnitude) {
				biggest_magnitude = biggest_magnitudes.ptr()[i];
			}
		}
		*/

		local_bounding_box.position = { 0, 0 };
		local_bounding_box.size = Vector2(2 + 2 * MAXQ_REAL_T_ARBITRARY_EPSILON, 2 + 2 * MAXQ_REAL_T_ARBITRARY_EPSILON);
		local_bounding_box.size *= biggest_magnitude; // calculated in previous parts of the code

		/*
		if (polygon_data_positions.size() > 0 && polygon_data_positions.ptr()[0].size() > 0) {
			local_bounding_box.set_position(polygon_data_positions.ptr()[0].ptr()[0]);
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

		// Increase it by 1.5 * so we also account for possible rotation
		// Yes it's approximate, shut
		local_bounding_box.size *= 1.5;
		*/

		//local_bounding_box.position = { 0, 0 };
		local_bounding_box.position = -local_bounding_box.size * 0.5;
	}

	// Here are the inertia calculations (we have the mass already)
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
			set_total_inertia(total_inertia); // also generates the inverse for future use
		}

	}

	// I set things like precomputed forces up so things are a lot faster, but the price we pay is having to update them whenever we change stuff
	// Thankfully, godot has this neat notification system which lets us notify everything "below" us about our predicament
	// This was mainly done so our precompute thrusters can recompute their precomputations but it lets anything catch the notification
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
	ClassDB::bind_method(D_METHOD("get_total_mass"), &PhysicsEntity2D::get_total_mass);
	ClassDB::bind_method(D_METHOD("get_inverse_total_mass"), &PhysicsEntity2D::get_inverse_total_mass);
	ClassDB::bind_method(D_METHOD("get_total_inertia"), &PhysicsEntity2D::get_total_inertia);
	ClassDB::bind_method(D_METHOD("get_inverse_total_inertia"), &PhysicsEntity2D::get_inverse_total_inertia);

	ClassDB::bind_method(D_METHOD("get_velocity"), &PhysicsEntity2D::get_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &PhysicsEntity2D::get_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_bounding_rect"), &PhysicsEntity2D::get_collision_box);

	ClassDB::bind_method(D_METHOD("apply_force", "global_location", "force"), &PhysicsEntity2D::apply_force);
	ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &PhysicsEntity2D::apply_central_force);
	ClassDB::bind_method(D_METHOD("apply_torque", "torque"), &PhysicsEntity2D::apply_torque);

	ClassDB::bind_method(D_METHOD("apply_impulse_force", "global_location", "impulse"), &PhysicsEntity2D::apply_impulse_force);
	ClassDB::bind_method(D_METHOD("apply_central_impulse_force", "impulse"), &PhysicsEntity2D::apply_central_impulse_force);
	ClassDB::bind_method(D_METHOD("apply_impulse_torque", "impulse_torque"), &PhysicsEntity2D::apply_impulse_torque);
	/*
	ClassDB::bind_method(D_METHOD("which_segment_contains_point", "global_point"), &PhysicsEntity2D::which_segment_contains_point);
	ClassDB::bind_method(D_METHOD("get_point_displacement_vec", "global_point", "direction_vec", "result"), &PhysicsEntity2D::get_point_displacement_vec);
	*/
}

// Solver and collision resolution stuff. Only the physics solver should call these
void PhysicsEntity2D::set_id(size_t index) {
	physics_id = index;
}

void PhysicsEntity2D::solver_apply_queued_forces(real_t delta_time) {
	if (queued_force.x != 0 || queued_force.y != 0) {
		queued_force = queued_force.rotated(get_global_rotation());

		queued_force *= delta_time;

		force_to_velocity(queued_force);
		queued_force = { 0, 0 };
	}

	if (queued_torque != 0) {
		queued_torque *= delta_time;

		torque_to_angular_velocity(queued_torque);
		queued_torque = 0;
	}
}

void PhysicsEntity2D::solver_prepare_for_step(real_t delta_time, real_t solver_linear_resolution, real_t solver_angular_resolution) {
	if (calculate_data_from_segments_next_step) {
		calculate_data_from_segments();
	}

	// Let's get our real bounding box ready
	{
		Vector2 position_delta = velocity * (delta_time - total_toi);

		real_bounding_box = local_bounding_box;
		real_bounding_box.position += get_global_position();
		future_bounding_box = real_bounding_box;
		future_bounding_box.position += position_delta;

		collision_bounding_box = real_bounding_box;
		collision_bounding_box.expand_to(future_bounding_box.position);
		collision_bounding_box.expand_to(future_bounding_box.get_end());
	}

	// Okay, now that the easy shit is done, we have to generate the inbetween states we're using for these collisions
	// It is in our best interest to keep this fairly performant
	{
		size_t num_of_segments = get_segments().size();
		//size_t data_order_len = data_order.size();

		real_t starting_angle = get_global_rotation();
		real_t starting_cos = cos(starting_angle);
		real_t starting_sin = sin(starting_angle);

		real_t angle_change = delta_time * get_angular_velocity();
		Vector2 position_change = delta_time * get_velocity();

		real_t end_angle = get_global_rotation() + angle_change;

		// First we have to check the number of timesteps
		// We assume everything else is correct because calculate_data_from_segments() handles the sizing and correctly sizes the first vec for us

		size_t linear_timesteps_to_generate = ceil(abs(position_change.length() * solver_linear_resolution));
		size_t angular_timesteps_to_generate = ceil(abs(angle_change / Math_PI * biggest_magnitude * solver_angular_resolution));

		size_t total_timesteps_to_generate = linear_timesteps_to_generate + angular_timesteps_to_generate;
		++total_timesteps_to_generate; // Add one for the start

		real_t delta_angle = angle_change / total_timesteps_to_generate;
		real_t delta_cos = cos(delta_angle);
		real_t delta_sin = sin(delta_angle);
		real_t delta_delta_time = delta_time / total_timesteps_to_generate;
		Vector2 delta_delta_velocity = position_change / total_timesteps_to_generate;

		++total_timesteps_to_generate; // And one for the end (we calc the above deltas with total - 1, but we have to add two at the end, so we add them seperately)

		for (size_t i = 0; i < num_of_segments; i++) {

			if (polygon_spacetime_data.ptrw()[i].size() < total_timesteps_to_generate) { // We don't really want to downsize
				size_t times_to_generate = total_timesteps_to_generate - polygon_spacetime_data.ptrw()[i].size();

				for (size_t j = 0; j < times_to_generate; j++) {
					polygon_spacetime_data.ptrw()[i].push_back(polygon_spacetime_data.ptr()[i].ptr()[0]);
				}
			}

			// Okay. Alright. Initial timestep now. I got this.
			// First we calculate everything without positional offset and then we apply it later

			size_t positions_vec_size = polygon_data_positions.ptr()[i].size();

			for (size_t p = 0; p < positions_vec_size; p++) {
				polygon_spacetime_data.ptrw()[i].ptrw()[0].ptrw()[p] =
						Vector3(polygon_data_positions.ptr()[i].ptr()[p].x * starting_cos - polygon_data_positions.ptr()[i].ptr()[p].y * starting_sin,
								polygon_data_positions.ptr()[i].ptr()[p].x * starting_sin + polygon_data_positions.ptr()[i].ptr()[p].y * starting_cos,
								0);
			}

			// Now we do the inbetween parts, we can just rotate the thing before it by an amount to avoid generating more sin and cos than we need to
			real_t next_delta_delta_time = delta_delta_time;

			for (size_t j = 0; j < total_timesteps_to_generate - 1; j++) {
				for (size_t p = 0; p < positions_vec_size; p++) {
					polygon_spacetime_data.ptrw()[i].ptrw()[j + 1].ptrw()[p] = // As total_timesteps_to_generate is decremented by one, we don't overflow
							Vector3(polygon_spacetime_data.ptr()[i].ptr()[j].ptr()[p].x * delta_cos - polygon_spacetime_data.ptr()[i].ptr()[j].ptr()[p].y * delta_sin,
									polygon_spacetime_data.ptr()[i].ptr()[j].ptr()[p].x * delta_sin + polygon_spacetime_data.ptr()[i].ptr()[j].ptr()[p].y * delta_cos,
									next_delta_delta_time);
									//delta_delta_time + j * delta_delta_time);
				}
				next_delta_delta_time += delta_delta_time;
			}

			// Now that The Rotationing is done, we can move everything by the appropriate amount and we're done
			Vector3 next_delta_delta_velocity = Vector3(0, 0, 0);

			for (size_t j = 0; j < total_timesteps_to_generate; j++) {
				for (size_t p = 0; p < positions_vec_size; p++) {
					polygon_spacetime_data.ptrw()[i].ptrw()[j].ptrw()[p] +=
							Vector3(get_global_position().x, get_global_position().y, 0) +
									next_delta_delta_velocity;
									//Vector3(j * delta_delta_velocity.x, j * delta_delta_velocity.y, 0);
				}
				next_delta_delta_velocity += Vector3(delta_delta_velocity.x, delta_delta_velocity.y, 0);
			}
		}
	}
}

void PhysicsEntity2D::solver_step_for_toi(real_t toi) {
	if (unlikely(toi < 0)) {
		print_error("We're going back in time @ PhysicsEntity2D::solver_step_for_toi!");
	}

	total_toi += toi;
	set_global_position(get_global_position() + velocity * toi);
	set_global_rotation(get_global_rotation() + angular_velocity * toi);
}

const Vector<Vector<Vector<Vector3>>> *PhysicsEntity2D::solver_get_spacetime_data() {
	return &polygon_spacetime_data;
}

void PhysicsEntity2D::solver_step(real_t delta_time) {

	//Vector<Vector<Vector2>> next_prediction;
	//load_all_final_points(next_prediction);

	real_t time = (delta_time - total_toi);
	set_global_position(get_global_position() + velocity * time);
	set_global_rotation(get_global_rotation() + angular_velocity * time);
	clear_collision_data();

	//Vector<Vector<Vector2>> actual_next_position;
	//load_all_current_points(actual_next_position);

	//print_line("Delta is " + (actual_next_position[0][0] - next_prediction[0][0]) + " out of " + (velocity * time));
}

void PhysicsEntity2D::clear_collision_data() {
	total_toi = 0;
	// Why was this here lol
	// data_order.clear();
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

	for (int i = length - 1; i >= 0; i--) {
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

size_t *PhysicsEntity2D::solver_get_polygon_data_vec_size() {
	return nullptr;
}

Vector<Vector<Vector2>> *PhysicsEntity2D::solver_get_direct_polygon_data_access(size_t index) {
	return nullptr;
}

/*
void PhysicsEntity2D::load_all_current_points(Vector<Vector<Vector2>> &target_vec) {
	size_t length = physics_segments.size();
	for (size_t i = 0; i < length; i++) {
		target_vec.push_back(physics_segments[i]->entity_get_global_currpos());
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
		target_vec.push_back(physics_segments[i]->entity_get_global_nextpos());
	}
}
*/
