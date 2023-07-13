/* component.cpp */

#include "thruster_movement_controller.h"

ThrusterMovementController::ThrusterMovementController() {
	tag = "bgThrustController";
	target_id = ComponentIDManager::get_singleton()->fetch_id("bgThruster");
}

void ThrusterMovementController::_notification(int p_what) {
	Component::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_PROCESS:
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			break;
	}
}
