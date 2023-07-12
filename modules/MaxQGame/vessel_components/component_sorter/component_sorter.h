/* component.h */

#pragma once

#include "scene/2d/node_2d.h"
#include "../components/component.h"

class Component; //<---- CIRCULAR DEPENDENCY

class ComponentSorter : public Node2D {
	GDCLASS(ComponentSorter, Node2D);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	Vector<Vector<Component *> > component_list;

	void add_component(Component *compptr);
	void remove_component(Component *compptr);
	const Vector<Component *> *get_component_vec(int comp_id);
	TypedArray<Node> get_component_list(int comp_id);

};
