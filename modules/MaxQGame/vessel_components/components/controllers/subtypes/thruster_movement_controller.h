/* component.h */

#pragma once

#include "../controller.h"

class ThrusterMovementController : public Controller {
	GDCLASS(ThrusterMovementController, Controller);

protected:
	void _notification(int p_what) override;

public:
	int target_id = 0;

	ThrusterMovementController();
};
