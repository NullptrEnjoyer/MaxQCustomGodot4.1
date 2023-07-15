/* component.h */

#pragma once

#include "../../controller.h"
#include"../../../subtypes/Thruster/thruster.h"

class ThrusterMovementController : public Controller {
	GDCLASS(ThrusterMovementController, Controller);

protected:
	bool full = false, front = false, back = false, left = false, right = false, rot_left = false, rot_right = false;

	void _notification(int p_what);
	static void _bind_methods();

	void reset_vars();

public:
	int target_id = 0;

	void move();

	void move_fullspeed();

	void move_front();
	void move_back();
	void move_left();
	void move_right();
	void rotate_left();
	void rotate_right();

	ThrusterMovementController();
};
