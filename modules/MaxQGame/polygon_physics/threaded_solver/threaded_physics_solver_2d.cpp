/* threaded_physics_solver_2d.cpp */

#include "threaded_physics_solver_2d.h"

void ThreadedPhysicsSolver2D::_notification(int p_what) {
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

void ThreadedPhysicsSolver2D::tick() {
	int length = entities.size();

	for (size_t i = 0; i < length; i++) {
		entities[i]->apply_queued_forces();
	}

	for (size_t i = 0; i < length; i++) {
		for (size_t j = 0; j < length; j++) {
			if (unlikely(i == j)) {
				continue;
			}
			entities[i]->check_collision(entities[j]);
		}
	}

	for (size_t i = 0; i < length; i++) {
		entities[i]->step();
	}
}

int ThreadedPhysicsSolver2D::add_entity(PhysicsEntity2D *detector) {
	int ret = entities.size();
	entities.push_back(detector);
	return ret;
}

void ThreadedPhysicsSolver2D::remove_entity(int detector_id) {
	entities.remove_at(detector_id);
}
