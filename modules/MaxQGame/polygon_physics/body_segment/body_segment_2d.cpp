/* body_segment_2d.cpp */

#include "body_segment_2d.h"

String PhysicsSegment2D::PhysicsEntityTypeID = typeid(PhysicsEntity2D).raw_name();

void PhysicsSegment2D::set_mass(real_t _mass) {
	if (entity != nullptr) {
		entity->calculate_data_from_segments();
	}
	mass = _mass;
}

real_t PhysicsSegment2D::get_mass() {
	return mass;
}

Vector2 PhysicsSegment2D::get_center_of_mass() {
	return center_of_mass;
}

bool PhysicsSegment2D::get_intersect_results_local(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset, Vector<Vector2> *results) {
	Vector<Vector2> polygon_vec = get_polygon();
	int vec_size = polygon_vec.size() - 1;

	Vector2 result = { 0, 0 };
	bool ret_val = false;

	for (int i = 0; i < vec_size; i++) {
		if (PhysicsEntity2D::solve_line_intersect_with_result(line1_point1, line1_point2, line1_offset, polygon_vec[i],
				polygon_vec[i + 1], Vector2{0,0}, &result)) {
			ret_val = true;
			results->push_back(result);
		}
	}

	return ret_val;
}

void PhysicsSegment2D::_bind_methods() {
	ADD_GROUP("Physics", "");
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &PhysicsSegment2D::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &PhysicsSegment2D::get_mass);
	ClassDB::add_property("PhysicsSegment2D", PropertyInfo(Variant::FLOAT, "mass"), "set_mass", "get_mass");
}

void PhysicsSegment2D::try_find_physics_entity() {
	Node *parent = get_parent();

	while (parent != NULL) {
		if (PhysicsEntityTypeID != typeid(*parent).raw_name()) {
			Node *new_parent = parent->get_parent();
			parent = new_parent;
			continue;
		}

		entity = static_cast<PhysicsEntity2D *>(parent);
		ID = entity->add_segment(this);
		return;
	}
}

void PhysicsSegment2D::try_remove_from_physics_entity() {
	if (entity != nullptr) {
		entity->remove_segment_at(ID);
		entity = nullptr;
	}
}

// https://stackoverflow.com/questions/5271583/center-of-gravity-of-a-polygon
// https://web.archive.org/web/20120229233701/http://paulbourke.net/geometry/polyarea/
// fails in self-intersecting polygons
void PhysicsSegment2D::calculate_area() {
	area = 0;

	Vector<Vector2> polygon_vec = get_polygon();
	int length = polygon_vec.size();

	// Skip-step first so we don't bother the for loop with checks
	size_t j = length - 1; // Current number here
	area += polygon_vec[j].x * polygon_vec[0].y;
	area -= polygon_vec[0].x * polygon_vec[j].y;

	length -= 1; // We just did that step
	for (size_t i = 0; i < length; i++) {
		j = i + 1; // The next number
		area += polygon_vec[i].x * polygon_vec[j].y;
		area -= polygon_vec[j].x * polygon_vec[i].y;
	}

	area /= 2;

	return;
}

void PhysicsSegment2D::calculate_center_of_mass() {
	center_of_mass = { 0, 0 };
	calculate_area();

	Vector<Vector2> polygon_vec = get_polygon();
	int length = polygon_vec.size();
	float factor = 0;

	// Skip-step first so we don't bother the for loop with checks
	size_t j = length - 1; // Current number here

	factor = (polygon_vec[j].x * polygon_vec[0].y - polygon_vec[0].x * polygon_vec[j].y);
	center_of_mass.x += (polygon_vec[j].x + polygon_vec[0].x) * factor;
	center_of_mass.y += (polygon_vec[j].y + polygon_vec[0].y) * factor;

	length -= 1; // We just did that step
	for (size_t i = 0; i < length; i++) {
		j = i + 1; // The next number
		factor = (polygon_vec[i].x * polygon_vec[j].y - polygon_vec[j].x * polygon_vec[i].y);
		center_of_mass.x += (polygon_vec[i].x + polygon_vec[j].x) * factor;
		center_of_mass.y += (polygon_vec[i].y + polygon_vec[j].y) * factor;
	}

	center_of_mass /= (area * 6);
}

void PhysicsSegment2D::_notification(int p_what) {
	Polygon2D::_notification(p_what);

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_READY: {
			calculate_center_of_mass();
			try_find_physics_entity();
		}break;

		case NOTIFICATION_PARENTED: {
			if (is_ready()) {
				try_find_physics_entity();
			}
		} break;

		case NOTIFICATION_UNPARENTED: {
			if (is_ready()) {
				try_remove_from_physics_entity();
			}
		} break;

		case NOTIFICATION_PREDELETE:
		case NOTIFICATION_EXIT_TREE:
			try_remove_from_physics_entity();
			break;
	}
}
