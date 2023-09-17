/* independent_physics_solver_2d.cpp */

#include "independent_physics_solver_2d.h"

void IndependentPhysicsSolver2D::_notification(int p_what) {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_READY: {
			set_physics_process(true);
		} break;

		case NOTIFICATION_PHYSICS_PROCESS: {
			tick();
		} break;
	}
}

void IndependentPhysicsSolver2D::tick() {

#ifdef POLYPHYSICS_SHOULD_STEP_ON_COLLISION
	if (HALT) {
		if (Input::get_singleton()->is_action_pressed("ui_accept")) {
			HALT = false;
		} else {
			return;
		}
	}
#endif

	solver_physics_time = 1.0 / Engine::get_singleton()->get_physics_ticks_per_second();

	int length = entity_vec.size();

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_apply_queued_forces(solver_physics_time);
	}

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_prepare_for_step(solver_physics_time, get_solver_linear_resolution(), get_solver_angular_resolution());
	}

	// Gather collisions
	for (size_t i = 0; i < length; i++) {
		for (size_t j = 0; j < length; j++) {
			if (i == j) { // Yes, I intersect myself, go away
				continue;
			}
			if (check_collision_broadphase(i, j)) {
				add_collision_pair(i, j);
			}
		}
	}

	resolve_collision_pairs();
	clear_collision_data();

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_step(solver_physics_time);
	}
}


#ifdef POLYPHYSICS_SHOULD_VISUALIZE
void IndependentPhysicsSolver2D::collision_visualizer_clear_data() {
	line_beginnings.clear();
	line_endings.clear();
	line_data.clear();

	for (size_t i = 0; i < visdat_objects_generated; i++) {
		Line2D *line = lines[i];
		line->clear_points();
		line->set_global_position(Vector2(0, 0));

		Label *label = line_labels[i];
		label->set_text("");
	}

	visdat_objects_used = 0;
}
void IndependentPhysicsSolver2D::collision_visualizer_add_line(Color _color, real_t _width, String _data, Vector2 _beginning, Vector2 _end) {
	Line2D *line;
	Label *label;

	if (visdat_objects_used >= visdat_objects_generated) {
		line = memnew(Line2D);
		add_child(line);
		line->set_owner(this);

		label = memnew(Label);
		line->add_child(label);
		label->set_owner(this);

		lines.push_back(line);
		line_labels.push_back(label);

		++visdat_objects_generated;
	} else {
		line = lines[visdat_objects_used];
		label = line_labels[visdat_objects_used];
	}
	++visdat_objects_used;

	line->set_default_color(_color);
	line->set_width(_width);
	label->set_text(_data);
	line->add_point(_beginning);
	line->add_point(_end);
}
void IndependentPhysicsSolver2D::collision_visualizer_generate_data() {
	collision_visualizer_clear_data();

	collision_visualizer_add_line({ 0, 0, 0, 255 }, 4, "", rep_collision_point - Vector2(2, 0), rep_collision_point + Vector2(2, 0));
	collision_visualizer_add_line({ 200, 0, 0, 255 }, 2, "", rep_collision_point, rep_collision_point + rep_collision_normal * 20);
}
#endif

real_t IndependentPhysicsSolver2D::get_solver_physics_time() {
	return solver_physics_time;
}

real_t IndependentPhysicsSolver2D::get_solver_linear_resolution() {
	return solver_linear_resolution;
}

real_t IndependentPhysicsSolver2D::get_solver_angular_resolution() {
	return solver_angular_resolution;
}

void IndependentPhysicsSolver2D::add_entity(PhysicsEntity2D *entity) {
	entity->set_id(entity_vec.size());
	entity_vec.push_back(entity);
}

void IndependentPhysicsSolver2D::remove_entity(int entity_id) {
	//We really don't care about sorting this mess, do it the fast way!
	int end_pos = entity_vec.size() - 1;
	entity_vec.set(entity_id, entity_vec.get(end_pos));
	entity_vec[entity_id]->set_id(entity_id);
	entity_vec.resize(end_pos); // Yeet that mfer
}

void IndependentPhysicsSolver2D::add_collision_pair(size_t body1, size_t body2) {
	real_t initial_toi = 0;

	size_t loc = collision_pair_vec.size();
	size_t id = collision_pair_id_vec.size();

	collision_pair_id_vec.push_back(loc);
	collision_pair_vec.push_back(CollisionPair(id, body1, body2, entity_vec[body1]->get_total_toi(), entity_vec[body2]->get_total_toi()));

	entity_vec[body1]->solver_add_collision_pair(id);
	entity_vec[body2]->solver_add_collision_pair(id);
}

bool IndependentPhysicsSolver2D::collision_pair_already_has_data(size_t collision_pair_id, size_t body1, size_t body2) {
	CollisionPair target_pair = collision_pair_vec[collision_pair_id_vec[collision_pair_id]];
	return ((target_pair.body1 == body1 && target_pair.body2 == body2) || (target_pair.body1 == body2 && target_pair.body2 == body1));
}

void IndependentPhysicsSolver2D::remove_collision_pair(size_t collision_pair_id) {
	size_t loc = collision_pair_id_vec[collision_pair_id];
	entity_vec[collision_pair_vec[loc].body1]->solver_remove_collision_pair(collision_pair_id);
	entity_vec[collision_pair_vec[loc].body2]->solver_remove_collision_pair(collision_pair_id);

	int end_pos = collision_pair_vec.size() - 1;

	collision_pair_vec.set(loc, collision_pair_vec.get(end_pos)); // We don't care about this being sorted, and this is a lot cheaper than remove_at due to our use case
	collision_pair_id_vec.set(collision_pair_vec.get(loc).ID, loc); // Its position is different, so we have to update the ID
	collision_pair_vec.resize(end_pos); // Yeet that mfer
}

// This function WILL NOT report a collision if it can already be found via our connected_collision_pair_index list
bool IndependentPhysicsSolver2D::check_collision_broadphase(size_t first_entity_loc, size_t second_entity_loc) {
	if (entity_vec[first_entity_loc]->collision_bounding_box.intersects(entity_vec[second_entity_loc]->collision_bounding_box)) {
		// If this already exists, return false. If we don't, return true.
		return !(entity_vec[first_entity_loc]->solver_collision_pair_already_exists(second_entity_loc));
	}
	return false;
}

bool IndependentPhysicsSolver2D::check_collision_narrowphase(size_t collision_pair_loc_not_id) {
	return false;
}

/*
// Find the collision with the smallest toi
bool IndependentPhysicsSolver2D::check_collision_narrowphase(size_t collision_pair_loc_not_id) {
	CollisionPair target_copy = collision_pair_vec[collision_pair_loc_not_id];

	PhysicsEntity2D *body1 = entity_vec[target_copy.body1];
	PhysicsEntity2D *body2 = entity_vec[target_copy.body2];

	real_t body1_starting_time = body1->get_total_toi();
	real_t body2_starting_time = body2->get_total_toi();

	Vector<Vector<Vector2>> body1_starting_points;
	Vector<Vector<Vector2>> body1_ending_points;
	Vector<Vector<Vector2>> body2_starting_points;
	Vector<Vector<Vector2>> body2_ending_points;

	if (body1_starting_time == body2_starting_time) {
		body1->load_all_current_points(body1_starting_points);
		body2->load_all_current_points(body2_starting_points);
	}
	else if (body1_starting_time > body2_starting_time) {
		body1->load_all_current_points(body1_starting_points);
		body2->load_all_specified_timestep_points(body2_starting_points, body1_starting_time);
	}
	else if (body1_starting_time < body2_starting_time) {
		body1->load_all_specified_timestep_points(body1_starting_points, body2_starting_time);
		body2->load_all_current_points(body2_starting_points);
	}

	body1->load_all_final_points(body1_ending_points);
	body2->load_all_final_points(body2_ending_points);

	size_t body1_segments_len = body1_starting_points.size();
	size_t body2_segments_len = body2_starting_points.size();

	LinePlaneIntersectResult3D final_result;
	bool returnbool = false;

	if (narrowphase_collision_compute(body1_starting_points, body1_ending_points, body2_starting_points, body2_ending_points,
				final_result, target_copy.additional_info)) {

		target_copy.body2_is_intruding_body = false;
		returnbool = true;
	}
	// Will return true ONLY if it finds a collision which happens sooner (has a lower param_t) than the provided result
	if (narrowphase_collision_compute(body2_starting_points, body2_ending_points, body1_starting_points, body1_ending_points,
				final_result, target_copy.additional_info)) {

		target_copy.body2_is_intruding_body = true;
		returnbool = true;
	}

	if (returnbool == true) {
		target_copy.toi = final_result.param_t * (get_physics_process_delta_time() - target_copy.initial_toi) + target_copy.initial_toi;
		target_copy.impact_normal = Vector2(final_result.intersect_normal.x, final_result.intersect_normal.y);
		target_copy.impact_point = Vector2(final_result.intersect_point.x, final_result.intersect_point.y);
		collision_pair_vec.set(collision_pair_loc_not_id, target_copy);

		return true;
	}
	return false;
}
*/

bool IndependentPhysicsSolver2D::narrowphase_collision_compute(Vector<Vector<Vector2>> &body1_starting_points_vec, Vector<Vector<Vector2>> &body1_ending_points_vec,
		Vector<Vector<Vector2>> &body2_starting_points_vec, Vector<Vector<Vector2>> &body2_ending_points_vec,
		LinePlaneIntersectResult3D &result, CollisionInfo &info)
{

	size_t body1_segments_len = body1_starting_points_vec.size();
	size_t body2_segments_len = body2_starting_points_vec.size();
	bool returnbool = false;

	// For every point we project where it goes in 3D space (time is the third dimension), and calculate the intersect between the moving point and the moving line
	// on the other body. Repeat this for both bodies and find the closest param_t, which represents how far (from 1 to 0) we went in time.
	// Save that as our collision and keep going until we checked all of them - the last collision standing is our result.
	// s means segment, p means point, i and j are the first and second iterator

	for (size_t si = 0; si < body1_segments_len; si++) {
		// THIS IS ONLY SAFE BECAUSE I KNOW WHAT THIS IS AND WHERE IT COMES FROM, AVOID DE-CONSTING WHERE YOU CAN
		Vector<Vector2> *body1_temp_starting_points_vec = const_cast<Vector<Vector2> *>(&body1_starting_points_vec.ptr()[si]);
		Vector<Vector2> *body1_temp_ending_points_vec = const_cast<Vector<Vector2> *>(&body1_ending_points_vec.ptr()[si]);
		size_t body1_temp_points_len = body1_temp_starting_points_vec->size();

		for (size_t pi = 0; pi < body1_temp_points_len; pi++) {
			for (size_t sj = 0; sj < body2_segments_len; sj++) {
				// THIS IS ONLY SAFE BECAUSE I KNOW WHAT THIS IS AND WHERE IT COMES FROM, AVOID DE-CONSTING WHERE YOU CAN
				Vector<Vector2> *body2_temp_starting_points_vec = const_cast<Vector<Vector2> *>(&body2_starting_points_vec.ptr()[sj]);
				Vector<Vector2> *body2_temp_ending_points_vec = const_cast<Vector<Vector2> *>(&body2_ending_points_vec.ptr()[sj]);
				size_t body2_temp_points_len = body2_temp_starting_points_vec->size();

				//Here we start from one because we actually do two locations at a time. We also do first - last after the for loop
				for (size_t pj = 1; pj < body2_temp_points_len; pj++) {
					if (narrowphase_collision_do_compute(
								body1_temp_starting_points_vec->get(pi), body1_temp_ending_points_vec->get(pi),
								body2_temp_starting_points_vec->get(pj - 1), body2_temp_starting_points_vec->get(pj),
								body2_temp_ending_points_vec->get(pj - 1), body2_temp_ending_points_vec->get(pj),
								result)) {
						returnbool = true;
						info.intruding_body_segment = si;
						info.intruding_body_point = pi;
						info.defending_body_segment = sj;
						info.defending_body_line_point_1 = pj - 1;
						info.defending_body_line_point_2 = pj;
					}
				}
				// And here we do the first-to-last points to complete the polygon
				size_t last_body2_point = body2_temp_points_len - 1;
				if (narrowphase_collision_do_compute(
							body1_temp_starting_points_vec->get(pi), body1_temp_ending_points_vec->get(pi),
							body2_temp_starting_points_vec->get(last_body2_point), body2_temp_starting_points_vec->get(0),
							body2_temp_ending_points_vec->get(last_body2_point), body2_temp_ending_points_vec->get(0),
							result)) {
					returnbool = true;
					info.intruding_body_segment = si;
					info.intruding_body_point = pi;
					info.defending_body_segment = sj;
					info.defending_body_line_point_1 = last_body2_point;
					info.defending_body_line_point_2 = 0;
				}
			}
		}
	}
	return returnbool;
}

bool IndependentPhysicsSolver2D::narrowphase_collision_do_compute(Vector2 line1point0, Vector2 line1point1,
		Vector2 plane_point_bottom_first, Vector2 plane_point_bottom_second, Vector2 plane_point_top_first, Vector2 plane_point_top_second,
		LinePlaneIntersectResult3D &result)
{

	bool returnbool = false;
	LinePlaneIntersectResult3D temp_result; // The function writes to this regardless of if it passes, so we must have the temp result go first

	if (solve_line_plane_collision_with_result(
				Vector3(line1point0.x, line1point0.y, 0), // Starting point
				Vector3(line1point1.x - line1point0.x,
						line1point1.y - line1point0.y, 1),// Direction vector

				Vector3(plane_point_bottom_first.x, plane_point_bottom_first.y, 0), // Starting point
				Vector3(plane_point_top_first.x - plane_point_bottom_first.x,
						plane_point_top_first.y - plane_point_bottom_first.y, 1), // Direction vectors
				Vector3(plane_point_top_second.x - plane_point_bottom_first.x,
						plane_point_top_second.y - plane_point_bottom_first.y, 1),

				&temp_result)) {
		// If u + v <= 1 the intersect was within the trinagle formed by the points, this is important due to rotation, and is also why we run this twice
		if (((temp_result.param_u + temp_result.param_v) <= (1 + MAXQ_REAL_T_ARBITRARY_EPSILON)) &&
				(!result.was_processed || (temp_result.param_t >= result.param_t))) {

			result.equalize(&temp_result);
			returnbool = true;
		}
	}
	if (solve_line_plane_collision_with_result(
				Vector3(line1point0.x, line1point0.y, 0),
				Vector3(line1point1.x - line1point0.x,
						line1point1.y - line1point0.y, 1),
				// Differences from previous if are below. Both might be true under some conditions so we do have to check both and find the smallest t
				Vector3(plane_point_bottom_second.x, plane_point_bottom_second.y, 0),
				Vector3(plane_point_bottom_first.x - plane_point_bottom_second.x,
						plane_point_bottom_first.y - plane_point_bottom_second.y, 0),
				Vector3(plane_point_top_second.x - plane_point_bottom_second.x,
						plane_point_top_second.y - plane_point_bottom_second.y, 1),

				&temp_result)) {
		// Ok yes I'm repeating myself, at least in this part. I'm sure its fine tho, and I don't want even more functions around
		if (((temp_result.param_u + temp_result.param_v) <= (1 + MAXQ_REAL_T_ARBITRARY_EPSILON)) &&
				(!result.was_processed || (temp_result.param_t >= result.param_t))) {

			result.equalize(&temp_result);
			returnbool = true;
		}
	}

	return returnbool;
}

void IndependentPhysicsSolver2D::resolve_collision_pairs() {
	size_t collision_pair_vec_length = collision_pair_vec.size();
	LinePlaneIntersectResult3D result;
	CollisionPair target;

	// First generate our data. We ignore initial TOI (time of impact) at first as we're starting from zero.
	for (size_t i = 0; i < collision_pair_vec_length; i++) {
		target = collision_pair_vec[i];

		if (!check_collision_narrowphase(i)) {
			remove_collision_pair(target.ID);
			--i;
			--collision_pair_vec_length;
			continue;
		}
		print_line("COLLISION found @ frame " + String::num_int64(Engine::get_singleton()->get_physics_frames()));
		//collision_pair_vec.set(i, target);
	}

	// return; // For now

	// Now we process until we have no more collision pairs. Might cause an infinite loop under some collision conditions, so I might have to fix that.
	{
		size_t prev_collision_pair_vec_length = 0;
		size_t body_1_index = 0;
		size_t body_2_index = 0;
		size_t entity_vec_length = 0;

		while (collision_pair_vec.size() != 0) {
			collision_pair_vec_length = collision_pair_vec.size(); // TECHNICALLY not needed at first pass, but it'd be cool if you got off my back about it
			int smallest_toi_collision_pair_index = 0;

			for (size_t i = 1; i < collision_pair_vec_length; i++) {
				if (collision_pair_vec[i].toi < collision_pair_vec[i - 1].toi) {
					smallest_toi_collision_pair_index = i - 1;
				}
			}

			body_1_index = collision_pair_vec[smallest_toi_collision_pair_index].body1;
			body_2_index = collision_pair_vec[smallest_toi_collision_pair_index].body2;
			// Removes all associated collisions once it's done as they are invalid
			// Regenerates broadphase collision boxes
			// Steps involved bodies ahead by their TOI
			resolve_collision(smallest_toi_collision_pair_index);
			entity_vec_length = entity_vec.size();

			prev_collision_pair_vec_length = collision_pair_vec.size();

			for (size_t i = 0; i < entity_vec_length; i++) {
				if (i != body_1_index && check_collision_broadphase(body_1_index, i)) {
					add_collision_pair(body_1_index, i);
				}
				if (i != body_2_index && check_collision_broadphase(body_2_index, i)) {
					add_collision_pair(body_2_index, i);
				}
			}

			// Now we only do narrowphase checks on every new collision pair, which is every vector between prev_collision_pair_vec_length and collision_pair_vec_length
			collision_pair_vec_length = collision_pair_vec.size();
			for (size_t i = prev_collision_pair_vec_length; i < collision_pair_vec_length; i++) {
				target = collision_pair_vec[i];

				if (!check_collision_narrowphase(i)) {
					remove_collision_pair(target.ID);
					--i;
					--collision_pair_vec_length;
					continue;
				}
				//collision_pair_vec.set(i, target);
			}
		}
	}
}

void IndependentPhysicsSolver2D::clear_collision_data() {
	collision_pair_id_vec.clear();
	collision_pair_vec.clear();

	
	int length = entity_vec.size();
	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_clear_collision_pair_vec();
	}
}

// https://en.m.wikipedia.org/wiki/Collision_response
// Slightly modified for 2D
void IndependentPhysicsSolver2D::resolve_collision(int collision_pair_loc) {
	print_line("Collision resolution fired, details to follow:");
	CollisionPair target_copy = collision_pair_vec[collision_pair_loc];

	PhysicsEntity2D *intruding_body;
	PhysicsEntity2D *defending_body;

	if (!target_copy.body2_is_intruding_body) {
		intruding_body = entity_vec[target_copy.body1];
		defending_body = entity_vec[target_copy.body2];
	} else {
		intruding_body = entity_vec[target_copy.body2];
		defending_body = entity_vec[target_copy.body1];
	}

	// TODO!!!
	print_error("Aren't you forgetting something? TODO!!!");

	// Important: these also reset position data. Do not disable.
	// intruding_body->solver_step_for_toi(target_copy.toi - intruding_body->get_total_toi());
	// defending_body->solver_step_for_toi(target_copy.toi - defending_body->get_total_toi());

	PhysicsSegment2D *intruding_body_affected_segment = intruding_body->get_segments().get(target_copy.additional_info.intruding_body_segment);
	PhysicsSegment2D *defending_body_affected_segment = defending_body->get_segments().get(target_copy.additional_info.defending_body_segment);

	// "Coefficient of restitution", basically the bouncyness of the objects
	real_t restitution_coefficient = (intruding_body_affected_segment->get_bouncyness() + defending_body_affected_segment->get_bouncyness()) / 2;
	//restitution_coefficient = -0.9;

	//The global position of a body is always its center of mass
	Vector2 intruding_center_of_mass_vec = target_copy.impact_point - intruding_body->get_global_position();
	Vector2 defending_center_of_mass_vec = target_copy.impact_point - defending_body->get_global_position();

	Vector2 impact_normal = target_copy.impact_normal;
	impact_normal.normalize();
	impact_normal *= -1;

	Vector2 intruding_angular_velocity_in_point = intruding_body->get_angular_velocity() * Vector2(-intruding_center_of_mass_vec.y, intruding_center_of_mass_vec.x);
	Vector2 defending_angular_velocity_in_point = defending_body->get_angular_velocity() * Vector2(-defending_center_of_mass_vec.y, defending_center_of_mass_vec.x);

	Vector2 velocity_differential = intruding_body->get_velocity() + intruding_angular_velocity_in_point -
			(defending_body->get_velocity() + defending_angular_velocity_in_point);

	real_t defending_subcomponent = (defending_body->get_inverse_total_inertia()) * (defending_center_of_mass_vec.cross(impact_normal));
	Vector2 defending_component = defending_subcomponent * Vector2(-defending_center_of_mass_vec.y, defending_center_of_mass_vec.x);

	real_t intruding_subcomponent = (intruding_body->get_inverse_total_inertia()) * (intruding_center_of_mass_vec.cross(impact_normal));
	Vector2 intruding_component = intruding_subcomponent * Vector2(-intruding_center_of_mass_vec.y, intruding_center_of_mass_vec.x);

	real_t needed_denominator_dot_product = (defending_component + intruding_component).dot(impact_normal);

	real_t numerator = -(((1 + restitution_coefficient) * (velocity_differential)).dot(impact_normal));

	real_t impulse_magnitude = numerator / ((defending_body->get_inverse_total_mass() + intruding_body->get_inverse_total_mass()) + needed_denominator_dot_product);
	Vector2 impulse = impulse_magnitude * impact_normal;

	/*
	print_line("defending_line" + defending_body_affected_segment->get_polygon().get(target_copy.additional_info.defending_body_line_point_1) +
		defending_body_affected_segment->get_polygon().get(target_copy.additional_info.defending_body_line_point_2));

	print_line("impact_point " + target_copy.impact_point);
	print_line("impact_normal " + impact_normal);
	print_line("impulse_magnitude " + String::num_real(impulse_magnitude));
	print_line("Impulse " + impulse);
	*/

#ifdef POLYPHYSICS_SHOULD_VISUALIZE
	rep_collision_point = target_copy.impact_point;
	rep_collision_normal = impact_normal * 5;

	rep_intruding_body_center = intruding_body->get_position();
	rep_intruding_body_velocity = intruding_body->get_velocity();
	rep_intruding_body_angular_velocity = intruding_body->get_angular_velocity();
	rep_intruding_body_momentum = rep_intruding_body_velocity * intruding_body->get_total_mass();
	rep_intruding_body_angular_momentum = rep_intruding_body_angular_velocity * intruding_body->get_total_inertia();

	rep_defending_body_center = defending_body->get_position();
	rep_defending_body_velocity = defending_body->get_velocity();
	rep_defending_body_angular_velocity = defending_body->get_angular_velocity();
	rep_defending_body_momentum = rep_defending_body_velocity * defending_body->get_total_mass();
	rep_defending_body_angular_momentum = rep_defending_body_angular_velocity * defending_body->get_total_inertia();

	rep_velocity_differential = { 0, 0 };
	rep_restitution_coefficient = 0;

	collision_visualizer_generate_data();
#endif

#ifdef POLYPHYSICS_SHOULD_STEP_ON_COLLISION
	HALT = true;
#endif

	defending_body->apply_impulse_force(target_copy.impact_point, -impulse);
	intruding_body->apply_impulse_force(target_copy.impact_point, impulse);

	// This code is 1000% safe and could never backfire catastrophically
	// The above is a joke, REMOVE THE EPSILON AND NORMAL AS SOON AS THE DISPLACER IS DONE!
	defending_body->set_global_position(defending_body->get_global_position() - MAXQ_REAL_T_ARBITRARY_EPSILON * impact_normal);
	intruding_body->set_global_position(intruding_body->get_global_position() + MAXQ_REAL_T_ARBITRARY_EPSILON * impact_normal);

	defending_body->solver_invalidate_collision_pair_vec();
	intruding_body->solver_invalidate_collision_pair_vec();

	// TODO!!!
	print_error("Aren't you forgetting something? TODO!!!");
	//defending_body->solver_prepare_for_step();
	//intruding_body->solver_prepare_for_step();
}
