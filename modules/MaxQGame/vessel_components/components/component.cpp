/* component.cpp */

#include "component.h"

String Component::sorter_typeid = typeid(ComponentSorter).raw_name();

void Component::get_id_from_manager() {
	id = ComponentIDManager::get_singleton()->fetch_id(tag);
}

void Component::try_add_to_sorter() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (sorter_typeid != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		ComponentSorter *sorter = static_cast<ComponentSorter *>(parent);
		sorter->add_component(this);
		curr_sorter = sorter;

		return;
	}

}

void Component::try_remove_from_sorter() {
	if (curr_sorter != nullptr) {
		curr_sorter->remove_component(this);
		curr_sorter = nullptr;
	}
}

void Component::_bind_methods() {

	ClassDB::bind_method(D_METHOD("get_id"), &Component::get_id);

	ADD_GROUP("Tag", "");
	ClassDB::bind_method(D_METHOD("set_tag", "new_tag"), &Component::set_tag);
	ClassDB::bind_method(D_METHOD("get_tag"), &Component::get_tag);
	ClassDB::add_property("Component", PropertyInfo(Variant::STRING, "tag"), "set_tag", "get_tag");
	/*
	Example:
	ClassDB::bind_method(D_METHOD("add", "value"), &Summator::add);// <- when it needs values
	ClassDB::bind_method(D_METHOD("reset"), &Summator::reset);// <- when it doesn't
	ClassDB::bind_method(D_METHOD("get_total"), &Summator::get_total);
	*/
}

void Component::_notification(int p_what) {

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		// Handled by NOTIFICATION_NODE_RECACHE_REQUESTED
		/* case NOTIFICATION_PARENTED: //first
			break;

		case NOTIFICATION_ENTER_TREE: //second
			break;*/

		case NOTIFICATION_READY://third
			get_id_from_manager();
			try_add_to_sorter();
			break;

		/* case NOTIFICATION_PROCESS:
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			break;*/

		case NOTIFICATION_NODE_RECACHE_REQUESTED:
			if (is_ready()) {
				try_remove_from_sorter();
				try_add_to_sorter();
			}
			break;

		// Handled by NOTIFICATION_NODE_RECACHE_REQUESTED
		/* case NOTIFICATION_UNPARENTED:
			break;*/

		case NOTIFICATION_PREDELETE:
		case NOTIFICATION_EXIT_TREE:
			try_remove_from_sorter();
			break;
	}
}

Component::Component() {
	tag = "bgBaseComponent";
}

String Component::get_tag() {
	return tag;
}

void Component::set_tag(String new_tag) {
	try_remove_from_sorter();
	tag = new_tag;
	get_id_from_manager();
	try_add_to_sorter();
}

int Component::get_id() {
	return id;
}
