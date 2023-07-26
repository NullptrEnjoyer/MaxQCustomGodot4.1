/* component_ID_singleton.h */

#pragma once

#include "core/object/class_db.h" // location of "Object", apparently
#include "core/templates/vector.h"
#include "core/os/mutex.h" // this can be accessed by multiple threads

class ComponentIDManager : public Object {
	GDCLASS(ComponentIDManager, Object);

protected:
	static ComponentIDManager *singleton;
	static void _bind_methods();

	struct ID {
		String str = "";
		int id = 0;

		friend bool operator==(const ID &l, const ID &r) {
			return (l.str == r.str);
		}

		friend bool operator<(const ID &l, const ID &r) {
			return (l.str < r.str);
		}

		friend bool operator>(const ID &l, const ID &r) {
			return (l.str > r.str);
		}
	};

	Vector<ID> id_list;

	// Multiple threads could access the manager's fetch_id, so we have to lock it down to avoid data fuckups
	Mutex mutex;

	int fetch_id_mutex(String name);

public:
	ComponentIDManager();
	static ComponentIDManager *get_singleton();

	int fetch_id(String name);
};
