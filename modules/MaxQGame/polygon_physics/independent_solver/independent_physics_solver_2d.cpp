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

	for (size_t i = 0; i < length; i++) {
		for (size_t j = 0; j < length; j++) {
			if (i == j) { // Yes, I intersect myself, go away
				continue;
			}
			if (entity_vec[i]->solver_check_collision_broadphase(entity_vec[j])) {

			}
		}
	}

	for (size_t i = 0; i < length; i++) {
		entity_vec[i]->solver_step();
	}

	clear_collision_data();
}

void IndependentPhysicsSolver2D::add_entity(PhysicsEntity2D *entity) {
	entity->set_id(entity_vec.size());
	entity_vec.push_back(entity);
}

void IndependentPhysicsSolver2D::remove_entity(int entity_id) {
	//We really don't care about sorting this mess
	int end_pos = entity_vec.size() - 1;
	entity_vec.set(entity_id, entity_vec.get(end_pos));
	entity_vec[entity_id]->set_id(entity_id);
	entity_vec.resize(end_pos); // Yeet that mfer
}

void IndependentPhysicsSolver2D::add_collision_pair(size_t body1, size_t body2) {
	real_t initial_toi = 0;
	real_t toi_b1 = entity_vec[body1]->get_total_toi();
	real_t toi_b2 = entity_vec[body2]->get_total_toi();

	if (toi_b1 > toi_b2) {
		initial_toi = toi_b1;
	} else {
		initial_toi = toi_b2;
	}

	size_t loc = collision_pair_vec.size();
	size_t id = collision_pair_id_vec.size();

	collision_pair_id_vec.push_back(loc);
	collision_pair_vec.push_back({ id, initial_toi, 0, body1, body2 });

	entity_vec[body1]->add_collision_pair(id);
	entity_vec[body2]->add_collision_pair(id);
}

bool IndependentPhysicsSolver2D::collision_pair_already_has_data(size_t collision_pair_id, size_t body1, size_t body2) {
	CollisionPair target_pair = collision_pair_vec[collision_pair_id_vec[collision_pair_id]];
	return ((target_pair.body1 == body1 && target_pair.body2 == body2) || (target_pair.body1 == body2 && target_pair.body2 == body1));
}

void IndependentPhysicsSolver2D::remove_collision_pair(size_t collision_pair_id) {
	size_t loc = collision_pair_id_vec[collision_pair_id];
	entity_vec[collision_pair_vec[loc].body1]->remove_collision_pair(collision_pair_id);
	entity_vec[collision_pair_vec[loc].body2]->remove_collision_pair(collision_pair_id);

	int end_pos = collision_pair_vec.size() - 1;

	collision_pair_vec.set(loc, collision_pair_vec.get(end_pos)); // We don't care about this being sorted, and this is a lot cheaper than remove_at due to our use case
	collision_pair_id_vec.set(collision_pair_vec.get(loc).ID, loc); // Its position is different, so we have to update the ID
}

void IndependentPhysicsSolver2D::clear_collision_data() {
	collision_pair_id_vec.clear();
	collision_pair_vec.clear();
}
