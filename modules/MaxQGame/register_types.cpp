/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"

/* singletons */

#include "vessel_components/component_ID_singleton/component_ID_singleton.h"

/* components */

#include "vessel_components/component_sorter/component_sorter.h"
#include "vessel_components/components/component.h"
#include "vessel_components/components/subtypes/thruster.h"

/* controllers */

#include "vessel_components/components/controllers/controller.h"
#include "vessel_components/components/controllers/subtypes/thruster_movement_controller.h"

static ComponentIDManager *ComponentIDSingletonPtr = NULL;

void initialize_MaxQGame_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	/* singletons */

	ClassDB::register_class<ComponentIDManager>();
	ComponentIDSingletonPtr = memnew(ComponentIDManager);
	Engine::get_singleton()->add_singleton(Engine::Singleton("ComponentIDManager", ComponentIDManager::get_singleton()));

	// Vessel components and such ahead

	ClassDB::register_class<ComponentSorter>();

	/* components */
	ClassDB::register_class<Component>();
	ClassDB::register_class<Thruster>();

	/* controllers */
	ClassDB::register_class<Controller>();
	ClassDB::register_class<ThrusterMovementController>();
}

void uninitialize_MaxQGame_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	// Nothing to do here in this example.

	//SIKE, get memory managed. Big thanks to whoever wrote the tutorial btw
	memdelete(ComponentIDSingletonPtr);
}
