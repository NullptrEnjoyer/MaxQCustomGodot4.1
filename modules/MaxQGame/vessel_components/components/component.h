/* component.h */

#pragma once

#include "scene\2d\node_2d.h"
#include "../component_sorter/component_sorter.h"
#include "../component_ID_singleton/component_ID_singleton.h"

class ComponentSorter; //<---- CIRCULAR DEPENDENCY

class Component : public Node2D {
	GDCLASS(Component, Node2D);

protected:
	static String sorter_typeid; //assigned in constructor via typeid

	void get_id_from_manager();
	void try_add_to_sorter();
	void try_remove_from_sorter();

	static void _bind_methods();
	virtual void _notification(int p_what);

	String tag = "";
	ComponentSorter *curr_sorter = nullptr;
	int id = 0; //fetched from the ID manager during ready / when changing tags and stuff

public:
	Component();

	String get_tag();
	void set_tag(String new_tag);
	int get_id();
	bool id_get = false;

};
