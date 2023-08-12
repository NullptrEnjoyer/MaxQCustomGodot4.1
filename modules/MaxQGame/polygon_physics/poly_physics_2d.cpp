/* poly_physics_2d.cpp */

#include "poly_physics_2d.h"

Vector4 LinePlaneIntersectResult3D::get_intersect_dtuv_data() {
	return Vector4(determinant, param_t, param_u, param_v);
}

Vector3 LinePlaneIntersectResult3D::get_intersect_point() {
	return intersect_point;
}

Vector3 LinePlaneIntersectResult3D::get_intersect_normal() {
	return intersect_normal;
}

void LinePlaneIntersectResult3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_intersect_dtuv_data"), &LinePlaneIntersectResult3D::get_intersect_dtuv_data);
	ClassDB::bind_method(D_METHOD("get_intersect_point"), &LinePlaneIntersectResult3D::get_intersect_point);
}

void LinePlaneIntersectResult3D::equalize(LinePlaneIntersectResult3D &other) {
	intersect_point = other.intersect_point;
	intersect_normal = other.intersect_normal;
	determinant = other.determinant;
	param_t = other.param_t;
	param_u = other.param_u;
	param_v = other.param_v;
}

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

// Usage of LinePlaneIntersectResult3D makes this perfectly GDScript compatible
bool PolygonPhysicsSystem2D::solve_line_plane_collision_with_result(Vector3 line1_point, Vector3 line1_dir,
		Vector3 plane1_point, Vector3 plane1_dir_u, Vector3 plane1_dir_v, LinePlaneIntersectResult3D *result_out) {

	if (unlikely(result_out == nullptr)) {
		print_error("result_out doesn't exist at PolygonPhysicsSystem2D::solve_line_plane_collision_with_result!");
		return false;
	}

	result_out->determinant = (-line1_dir).dot(plane1_dir_u.cross(plane1_dir_v));

	float one_over_det = 1 / result_out->determinant;
	Vector3 distance_diff = line1_point - plane1_point;

	result_out->intersect_normal = plane1_dir_u.cross(plane1_dir_v);

	result_out->param_t = one_over_det * (result_out->intersect_normal.dot(distance_diff));
	result_out->param_u = one_over_det * ((plane1_dir_v.cross(-line1_dir)).dot(distance_diff));
	result_out->param_v = one_over_det * (((-line1_dir).cross(plane1_dir_u)).dot(distance_diff));

	result_out->intersect_point = line1_point + result_out->param_t * line1_dir;

	return ((result_out->param_t >= 0 && result_out->param_t <= 1) &&
			(result_out->param_u >= 0 && result_out->param_u <= 1) &&
			(result_out->param_v >= 0 && result_out->param_v <= 1));
}

void PolygonPhysicsSystem2D::_bind_methods() {
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_with_result", "line1_point1", "line1_point2", "line1_offset",
					"line2_point1", "line2_point2", "line2_offset"), &PolygonPhysicsSystem2D::solve_line_intersect_with_result_GDS);
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_simple", "line1_point1", "line1_point2", "line1_offset",
					"line2_point1", "line2_point2", "line2_offset"),
			&PolygonPhysicsSystem2D::solve_line_intersect_simple);
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
			D_METHOD("solve_line_plane_collision_with_result", "line1_point1", "line1_dir_vec",
					"plane1_point1", "plane1_dir_u", "plane1_dir_v", "result_out"),
			&PolygonPhysicsSystem2D::solve_line_plane_collision_with_result);
}
