/* component.cpp */

#include "component_sorter.h"
#include <cstdint> // uintptr_t

void ComponentSorter::add_component(Component *compptr) {
	int id = compptr->get_id();

	if (id > component_list.size() - 1) {
		component_list.resize(id + 1);
	}

	const Vector<Component *> *constvec = &component_list.ptr()[id];
	Vector<Component *> *vec = reinterpret_cast<Vector<Component *> *>((uintptr_t)constvec);

	vec->push_back(compptr);

	return;

	// Using get() and modifying doesn't work, I think it does a copy or something (every get gave me a different pointer). Idk doesn't matter, it works now
	// Using std::vector just crashes lol
	// //Turns out you're supposed to also set() it
}

void ComponentSorter::remove_component(Component *compptr) {
	int id = compptr->get_id();

	const Vector<Component *> *constvec = &component_list.ptr()[id];
	Vector<Component *> *vec = reinterpret_cast<Vector<Component *> *>((uintptr_t)constvec);

	vec->erase(compptr);

	return;
}

const Vector<Component *> *ComponentSorter::get_component_vec(int comp_id) {
	if (comp_id > component_list.size() - 1) {
		component_list.resize(comp_id + 1);
	}

	return &(component_list.ptr()[comp_id]);// Warn my ass, this bish should never get deleted without whoever's accessing it getting deleted
}

TypedArray<Node> ComponentSorter::get_component_list(int comp_id) {
	if (comp_id > component_list.size() - 1) {
		component_list.resize(comp_id + 1);
	}

	TypedArray<Node> components;
	const Vector<Component *> *component_vec = get_component_vec(comp_id);

	int length = component_vec->size();
	for (size_t i = 0; i < length; i++) {
		components.push_back(component_vec->get(i));
	}

	return components;
}

void ComponentSorter::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_component", "component"), &ComponentSorter::add_component);
	ClassDB::bind_method(D_METHOD("remove_component", "component"), &ComponentSorter::remove_component);
	ClassDB::bind_method(D_METHOD("get_component_list", "component_id"), &ComponentSorter::get_component_list);
}
