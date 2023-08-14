/* body_segment_2d.cpp */

#include "body_segment_2d.h"
#include "../../defines.h"

String PhysicsSegment2D::PhysicsEntityTypeID = typeid(PhysicsEntity2D).raw_name();

void PhysicsSegment2D::set_mass(real_t _mass) {
	if (_mass == 0) {
		_mass = 1;
	}
	mass = _mass;
	try_refresh_physics_entity();
}

real_t PhysicsSegment2D::get_mass() {
	return mass;
}

real_t PhysicsSegment2D::get_area() {
	return area;
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

void PhysicsSegment2D::try_refresh_physics_entity() {
	if (entity != nullptr) {
		entity->queue_calculate_data_from_segments();
	}
}

// https://stackoverflow.com/questions/5271583/center-of-gravity-of-a-polygon
// https://web.archive.org/web/20120229233701/http://paulbourke.net/geometry/polyarea/
// fails in self-intersecting polygons
void PhysicsSegment2D::calculate_area() {
	area = 0;

	Vector<Vector2> polygon_vec = get_polygon();
	int length = polygon_vec.size();

	if (unlikely(length < 3)) {
		print_error("PhysicsSegment2D::calculate_area(): Polygon has no area, you sure you wanna do this?");
		return;
	}

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

	if (area < 0) { // Whoops lol. I dunno if this breaks things, but I'd rather have it going the right way 'round
		polygon_vec.reverse();
		set_polygon(polygon_vec);
		area *= -1;
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

	if (unlikely(length < 3)) {
		print_error("PhysicsSegment2D::calculate_center_of_mass(): Polygon has no mass, you sure you wanna do this?");
		return;
	}

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

Vector<Vector2> PhysicsSegment2D::entity_get_global_currpos() {
	if (likely(entity != nullptr)  && !currpos_generated) {
		currpos.append_array(get_polygon());
		size_t currpos_len = currpos.size();

		Vector2 segment_glob_pos = get_global_position();
		real_t segment_glob_rot = get_global_rotation();

		for (size_t i = 0; i < currpos_len; i++) {
			Vector2 gon = currpos.get(i);

			gon = gon.rotated(segment_glob_rot);
			gon += segment_glob_pos;

			currpos.set(i, gon);
		}

		currpos_generated = true;
	}

	return currpos;
}

Vector<Vector2> PhysicsSegment2D::entity_get_global_thispos(real_t target_time) {
	// Make sure they're both generated first
	entity_get_global_currpos();
	entity_get_global_nextpos();

	Vector<Vector2> target_pos;
	target_pos.append_array(currpos);

	size_t posvec_len = nextpos.size();
	size_t multip_factor = (target_time - (get_physics_process_delta_time() - generated_nextpos_for_time)) / generated_nextpos_for_time;
	for (size_t i = 0; i < posvec_len; i++) {
		target_pos.set(i, currpos.get(i) + (nextpos.get(i) - currpos.get(i)) * multip_factor);
	}

	return target_pos;
}

Vector<Vector2> PhysicsSegment2D::entity_get_global_nextpos() {
	if (likely(entity != nullptr) && (!nextpos_generated)) {
		Vector2 velocity = entity->get_velocity();
		real_t angular_velocity = entity->get_angular_velocity();
		real_t for_time = get_physics_process_delta_time() - entity->get_total_toi();

		nextpos.append_array(get_polygon());
		size_t nextpos_len = nextpos.size();

		Vector2 entity_glob_pos = entity->get_global_position();

		Vector2 segment_glob_pos = get_global_position();
		real_t segment_glob_rot = get_global_rotation();

		Vector2 delta_space = (velocity * for_time);

		/*
		if (delta_space.x < 0) {
			delta_space.x -= MAXQ_REAL_T_ARBITRARY_EPSILON;
		} else {
			delta_space.x += MAXQ_REAL_T_ARBITRARY_EPSILON;
		}
		if (delta_space.y < 0) {
			delta_space.y -= MAXQ_REAL_T_ARBITRARY_EPSILON;
		}
		else {
			delta_space.y += MAXQ_REAL_T_ARBITRARY_EPSILON;
		}*/

		real_t delta_rotation = (angular_velocity * for_time);

		for (size_t i = 0; i < nextpos_len; i++) {
			Vector2 gon = nextpos.get(i);

			gon = gon.rotated(segment_glob_rot);
			gon += segment_glob_pos - entity_glob_pos;
			gon = gon.rotated(delta_rotation);
			gon += entity_glob_pos + delta_space;

			//gon = gon.rotated(segment_next_glob_rot);
			//gon += segment_next_glob_pos;

			nextpos.set(i, gon);
		}
		nextpos_generated = true;
		generated_nextpos_for_time = for_time;
	}

	return nextpos;
}

void PhysicsSegment2D::entity_clear_position_data() {
	currpos_generated = false;
	currpos.clear();
	nextpos_generated = false;
	nextpos.clear();
}
