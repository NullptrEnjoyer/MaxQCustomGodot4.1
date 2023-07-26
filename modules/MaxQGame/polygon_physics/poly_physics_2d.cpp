/* poly_physics_2d.cpp */

#include "poly_physics_2d.h"

bool PolygonPhysicsSystem2D::solve_line_intersect_with_result(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset, Vector2 *intersect_point) {

	// Direction vectors
	Vector2 dir_vec_1 = line1_point2 - line1_point1;
	Vector2 dir_vec_2 = line2_point2 - line2_point1;

	line1_point1 += line1_offset;
	line2_point1 += line2_offset;

	real_t denominator = dir_vec_1.x * dir_vec_2.y - dir_vec_2.x * dir_vec_1.y;

	real_t t = (dir_vec_2.y * line2_point1.x - dir_vec_2.y * line1_point1.x - dir_vec_2.x * line2_point1.y + dir_vec_2.x * line1_point1.y) / denominator;
	real_t u = -(-dir_vec_1.y * line2_point1.x + dir_vec_1.y * line1_point1.x + dir_vec_1.x * line2_point1.y - dir_vec_1.x * line1_point1.y) / denominator;

	*intersect_point = line1_point1 + (t * dir_vec_1);

	if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
		return 1;
	}
	return 0;
}

Vector2 PolygonPhysicsSystem2D::solve_line_intersect_with_result_GDS(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset) {

	Vector2 pass_vector = { 0, 0 }; // Ran out of patience, this is shit
	bool result = solve_line_intersect_with_result(line1_point1, line1_point2, line1_offset, line2_point1, line2_point2, line2_offset, &pass_vector);
	return pass_vector;
}

bool PolygonPhysicsSystem2D::solve_line_intersect_simple(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset) {

	// Direction vectors
	Vector2 dir_vec_1 = line1_point2 - line1_point1;
	Vector2 dir_vec_2 = line2_point2 - line2_point1;

	line1_point1 += line1_offset;
	line2_point1 += line2_offset;

	real_t denominator = dir_vec_1.x * dir_vec_2.y - dir_vec_2.x * dir_vec_1.y;

	real_t t = (dir_vec_2.y * line2_point1.x - dir_vec_2.y * line1_point1.x - dir_vec_2.x * line2_point1.y + dir_vec_2.x * line1_point1.y) / denominator;
	real_t u = -(-dir_vec_1.y * line2_point1.x + dir_vec_1.y * line1_point1.x + dir_vec_1.x * line2_point1.y - dir_vec_1.x * line1_point1.y) / denominator;

	if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
		return 1;
	}
	return 0;
}

void PolygonPhysicsSystem2D::_bind_methods() {
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_with_result", "line1_point1", "line1_point2", "line1_offset",
					"line2_point1", "line2_point2", "line2_offset"), &PolygonPhysicsSystem2D::solve_line_intersect_with_result_GDS);
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_simple", "line1_point1", "line1_point2", "line1_offset",
		"line2_point1", "line2_point2", "line2_offset"), &PolygonPhysicsSystem2D::solve_line_intersect_simple);
}
