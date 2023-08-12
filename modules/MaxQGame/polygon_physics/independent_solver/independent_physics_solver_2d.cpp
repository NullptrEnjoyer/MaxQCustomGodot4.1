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
	int length = entity_vec.size();

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_apply_queued_forces();
	}

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_prepare_for_step();
	}

	// Gather collisions
	for (size_t i = 0; i < length; i++) {
		for (size_t j = 0; j < length; j++) {
			if (i == j) { // Yes, I intersect myself, go away
				continue;
			}
			if (check_collision_broadphase(i ,j)) {
				add_collision_pair(i, j);
				//print_line("Bounding box intersect found @ frame " + String::num_int64(Engine::get_singleton()->get_frames_drawn()));
			}
		}
	}

	//print_line(String::num(collision_pair_vec.size()));

	resolve_collision_pairs();
	clear_collision_data();

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_step();
	}

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

// Find the collision with the smallest toi
bool IndependentPhysicsSolver2D::check_collision_narrowphase(size_t collision_pair_loc_not_id) {
	CollisionPair target = collision_pair_vec[collision_pair_id_vec[collision_pair_loc_not_id]];

	PhysicsEntity2D *body1 = entity_vec[target.body1];
	PhysicsEntity2D *body2 = entity_vec[target.body1];

	real_t body1_starting_time = body1->get_total_toi();
	real_t body2_starting_time = body2->get_total_toi();

	Vector<PhysicsSegment2D*> body1_segment_vec = body1->get_segments();
	size_t body1_segment_vec_len = body1_segment_vec.size();
	Vector<PhysicsSegment2D *> body2_segment_vec = body2->get_segments();
	size_t body2_segment_vec_len = body2_segment_vec.size();

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
	LinePlaneIntersectResult3D temp_result;

	bool returnbool = false;

	if (narrowphase_collision_compute(body1_starting_points, body1_ending_points, body2_starting_points, body2_ending_points, final_result)) {
		returnbool = true;
	}
	if (narrowphase_collision_compute(body2_starting_points, body2_ending_points, body1_starting_points, body1_ending_points, temp_result)) {
		returnbool = true;
		if (temp_result.param_t > final_result.param_t) {
			final_result.equalize(temp_result);
		}
	}

	return returnbool;
}

bool IndependentPhysicsSolver2D::narrowphase_collision_compute(Vector<Vector<Vector2>> &body1_starting_points_vec, Vector<Vector<Vector2>> &body1_ending_points_vec,
			Vector<Vector<Vector2>> &body2_starting_points_vec, Vector<Vector<Vector2>> &body2_ending_points_vec, LinePlaneIntersectResult3D &result) {

	size_t body1_segments_len = body1_starting_points_vec.size();
	size_t body2_segments_len = body2_starting_points_vec.size();
	LinePlaneIntersectResult3D temp_result;
	bool returnbool = false;

	// For every point we project where it goes in 3D space (time is the third dimension), and calculate the intersect between the moving point and the moving line
	// on the other body. Repeat this for both bodies and find the closest param_t, which represents how far% we went in time.
	// Save that as our collision and keep going until we checked all of them - the last collision standing is our result.
	// s means segment, p means point, i and j are the first and second iterator

	for (size_t si = 0; si < body1_segments_len; si++) {
		// THIS IS ONLY SAFE BECAUSE I KNOW WHAT THIS IS AND WHERE IT COMES FROM, AVOID DE-CONSTING WHERE YOU CAN
		Vector<Vector2> *body1_temp_starting_points_vec = const_cast<Vector<Vector2> *>(&body1_starting_points_vec.ptr()[si]);
		Vector<Vector2> *body1_temp_ending_points_vec = const_cast<Vector<Vector2> *>(&body2_starting_points_vec.ptr()[si]);
		size_t body1_temp_points_len = body1_temp_starting_points_vec->size();

		for (size_t pi = 0; pi < body1_temp_points_len; pi++) {
			for (size_t sj = 0; sj < body2_segments_len; sj++) {
				// THIS IS ONLY SAFE BECAUSE I KNOW WHAT THIS IS AND WHERE IT COMES FROM, AVOID DE-CONSTING WHERE YOU CAN
				Vector<Vector2> *body2_temp_starting_points_vec = const_cast<Vector<Vector2> *>(&body1_starting_points_vec.ptr()[sj]);
				Vector<Vector2> *body2_temp_ending_points_vec = const_cast<Vector<Vector2> *>(&body2_starting_points_vec.ptr()[sj]);
				size_t body2_temp_points_len = body2_temp_starting_points_vec->size();

				//Here we start from one because we actually do two locations at a time. We also do first - last after the for loop
				for (size_t pj = 1; pj < body2_temp_points_len; pj++) {
					if (solve_line_plane_collision_with_result(
								Vector3(body1_temp_starting_points_vec->get(pi).x, body1_temp_starting_points_vec->get(pi).y, 0),
								Vector3(body1_temp_ending_points_vec->get(pi).x - body1_temp_starting_points_vec->get(pi).x,
										body1_temp_ending_points_vec->get(pi).y - body1_temp_starting_points_vec->get(pi).y, 1),

								Vector3(body2_temp_starting_points_vec->get(pj - 1).x, body2_temp_starting_points_vec->get(pj - 1).y, 0),
								Vector3(body2_temp_ending_points_vec->get(pj - 1).x - body2_temp_starting_points_vec->get(pj - 1).x,
									body2_temp_ending_points_vec->get(pj - 1).y - body2_temp_starting_points_vec->get(pj - 1).y, 1),
								Vector3(body2_temp_ending_points_vec->get(pj).x - body2_temp_starting_points_vec->get(pj - 1).x,
									body2_temp_ending_points_vec->get(pj).y - body2_temp_starting_points_vec->get(pj - 1).y, 1),

								&temp_result)) {
						if (temp_result.param_t >= result.param_t) {
							result.equalize(temp_result);
							returnbool = true;
						}
					}
				}
			}
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

		if (!check_collision_narrowphase(target.ID)) {
			//print_line("Wrong COLLISION found @ frame " + String::num_int64(Engine::get_singleton()->get_frames_drawn()));
			remove_collision_pair(target.ID);
			--i;
			--collision_pair_vec_length;
			continue;
		}
		print_line("COLLISION found @ frame " + String::num_int64(Engine::get_singleton()->get_frames_drawn()));
		collision_pair_vec.set(i, target);
	}

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

			prev_collision_pair_vec_length = collision_pair_vec_length;
			body_1_index = collision_pair_vec[smallest_toi_collision_pair_index].body1;
			body_2_index = collision_pair_vec[smallest_toi_collision_pair_index].body1;
			// Removes all associated collisions once it's done as they are invalid
			// Regenerates broadphase collision boxes
			// Steps involved bodies ahead by their TOI
			// resolve_collision(smallest_toi_collision_pair_index);
			entity_vec_length = entity_vec.size();

			if (check_collision_broadphase(body_1_index, body_2_index)) {
				add_collision_pair(body_1_index, body_2_index);
			}

			for (size_t i = 0; i < entity_vec_length; i++) {
				if (i == body_1_index || i == body_2_index) {
					continue;
				}
				if (check_collision_broadphase(body_1_index, i)) {
					add_collision_pair(body_1_index, i);
				}
				if (check_collision_broadphase(body_2_index, i)) {
					add_collision_pair(body_2_index, i);
				}
			}

			// Now we only do narrowphase checks on every new collision pair
			collision_pair_vec_length = collision_pair_vec.size();
			for (size_t i = prev_collision_pair_vec_length; i < collision_pair_vec_length; i++) {
				target = collision_pair_vec[i];

				if (!check_collision_narrowphase(target.ID)) {
					remove_collision_pair(target.ID);
					--i;
					--collision_pair_vec_length;
					continue;
				}
				collision_pair_vec.set(i, target);
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


/*
if (!entity_vec[collision_pair_vec[i].body1]->solver_check_collision_narrowphase(entity_vec[collision_pair_vec[i].body2], target.initial_toi, &result)) {
	remove_collision_pair(target.ID);
	--i; // Will never try to access -1 because this is size_t, therefore we overflow into a number bigger than the length if we're already 0
	--collision_pair_vec_length;
	continue;
}

//target.toi = target.initial_toi + result.param_t * (get_physics_process_delta_time() - target.initial_toi);
target.toi = result.param_t * get_physics_process_delta_time();
target.impact_point = Vector2(result.intersect_point.x, result.intersect_point.y);
target.impact_normal = Vector2(result.collision_normal.x, result.collision_normal.y);
*/
