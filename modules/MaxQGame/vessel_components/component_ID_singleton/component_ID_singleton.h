/* component.h */

#pragma once

#include "core/object/class_db.h"//location of "Object", apparently
#include "core/templates/vector.h"

class ComponentIDManager : public Object {
	GDCLASS(ComponentIDManager, Object);

public:
	ComponentIDManager();
	static ComponentIDManager *get_singleton();

	int fetch_id(String name);

protected:
	static ComponentIDManager *singleton;
	static void _bind_methods();

	struct ID {
		String str = "";
		int id = 0;
	};

	Vector<ID> id_list;
};
