/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"

/* singletons */

#include "vessel_components/component_ID_singleton/component_ID_singleton.h"

/* polygon physics */

#include "polygon_physics/poly_physics_2d.h"
#include "polygon_physics/physics_entity/physics_entity_2d.h"
#include "polygon_physics/independent_solver/independent_physics_solver_2d.h"
#include "polygon_physics/body_segment/body_segment_2d.h"
#include "polygon_physics/line_intersect_sensor/line_intersect_sensor_2d.h"

/* components */

#include "vessel_components/component_sorter/component_sorter.h"
#include "vessel_components/components/component.h"
#include "vessel_components/components/subtypes/thruster/thruster.h"

/* controllers */

#include "vessel_components/components/controllers/controller.h"
#include "vessel_components/components/controllers/subtypes/thruster_movement_controller/thruster_movement_controller.h"

static ComponentIDManager *ComponentIDSingletonPtr = NULL;

void initialize_MaxQGame_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	/* singletons */

	ClassDB::register_class<ComponentIDManager>();
	ComponentIDSingletonPtr = memnew(ComponentIDManager);
	Engine::get_singleton()->add_singleton(Engine::Singleton("ComponentIDManager", ComponentIDManager::get_singleton()));

	/* polygon physics */

	GDREGISTER_CLASS(LinePlaneIntersectResult3D);
	GDREGISTER_VIRTUAL_CLASS(PolygonPhysicsSystem2D);
	GDREGISTER_CLASS(PhysicsEntity2D);
	GDREGISTER_CLASS(IndependentPhysicsSolver2D);
	GDREGISTER_CLASS(PhysicsSegment2D);
	GDREGISTER_CLASS(LineIntersectSensor2D);

	// Vessel components and such ahead

	ClassDB::register_class<ComponentSorter>();

	/* components */
	GDREGISTER_VIRTUAL_CLASS(Component);
	ClassDB::register_class<Thruster>();

	/* controllers */
	GDREGISTER_VIRTUAL_CLASS(Controller);
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
