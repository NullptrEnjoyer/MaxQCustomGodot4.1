/* component.cpp */

#include "component_ID_singleton.h"

ComponentIDManager *ComponentIDManager::singleton;

ComponentIDManager::ComponentIDManager() {
	singleton = this;
}

ComponentIDManager *ComponentIDManager::get_singleton() {
	return singleton;
}

int ComponentIDManager::fetch_id(String name) {
	int length = id_list.size();

	for (size_t i = 0; i < length; i++) {
		if (id_list[i].str == name) {
			return id_list[i].id;
		}
	}

	id_list.push_back({name, length});

	return length;
}

void ComponentIDManager::_bind_methods() {
	ClassDB::bind_method(D_METHOD("fetch_id", "name"), &ComponentIDManager::fetch_id);
}
