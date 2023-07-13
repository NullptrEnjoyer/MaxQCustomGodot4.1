/* component.cpp */

#include "controller.h"

Controller::Controller() {
	tag = "bgBaseController";
}

/* Example I guess
void Controller::_notification(int p_what) {
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
*/
