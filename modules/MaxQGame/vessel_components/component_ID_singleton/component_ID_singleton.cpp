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

	int from = 0;
	int to = length - 1;
	int midpoint = 0;

	// Slight help from https://www.geeksforgeeks.org/binary-search/
	while (from <= to) {

		midpoint = ((to - from) >> 1) + from; // Blitz approximate div by 2, number never negative

		if (name == id_list[midpoint].str) {
			return id_list[midpoint].id;
		}

		if (name > id_list[midpoint].str) {
			from = midpoint + 1;
		} else {
			to = midpoint - 1;
		}
	}

	// We have to add a new id, go thread-safe and retry - we must make sure no other thread is trying to do the same thing
	// This houldn't happen often
	return fetch_id_mutex(name);
}

int ComponentIDManager::fetch_id_mutex(String name) {

	int length = id_list.size();

	if (mutex.try_lock()) { // If it's already unlocked there's really no reason to wait
		id_list.ordered_insert({ name, length });
		mutex.unlock();
		return length;
	}

	int from = 0;
	int to = length - 1;
	int midpoint = 0;

	mutex.lock();
	// IF YOU ARE ADDING ADDITIONAL RETURNS, ALWAYS UNLOCK OR YOU WILL CAUSE A DEADLOCK

	// same thing again
	while (from <= to) {
		midpoint = ((to - from) >> 1) + from; // Blitz approximate div by 2, number never negative

		if (name == id_list[midpoint].str) {
			mutex.unlock();
			return id_list[midpoint].id;
		}

		if (name > id_list[midpoint].str) {
			from = midpoint + 1;
		} else {
			to = midpoint - 1;
		}
	}

	id_list.ordered_insert({ name, length });
	mutex.unlock();
	return length;
}

void ComponentIDManager::_bind_methods() {
	ClassDB::bind_method(D_METHOD("fetch_id", "name"), &ComponentIDManager::fetch_id);
}
