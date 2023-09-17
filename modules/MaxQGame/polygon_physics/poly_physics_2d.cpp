/* poly_physics_2d.cpp */

#include "poly_physics_2d.h"
#include "../defines.h"

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
	ClassDB::bind_method(D_METHOD("get_intersect_normal"), &LinePlaneIntersectResult3D::get_intersect_normal);
	ClassDB::bind_method(D_METHOD("equalize", "other"), &LinePlaneIntersectResult3D::equalize);
}

void LinePlaneIntersectResult3D::equalize(LinePlaneIntersectResult3D *other) {
	intersect_point = other->intersect_point;
	intersect_normal = other->intersect_normal;
	determinant = other->determinant;
	param_t = other->param_t;
	param_u = other->param_u;
	param_v = other->param_v;
	was_processed = other->was_processed;
}

real_t VectorVectorIntersectResult2D::get_param_t() {
	return param_t;
}

real_t VectorVectorIntersectResult2D::get_param_u() {
	return param_u;
}

void VectorVectorIntersectResult2D::equalize(VectorVectorIntersectResult2D *other) {
	param_t = other->get_param_t();
	param_u = other->get_param_u();
}

void VectorVectorIntersectResult2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_param_t"), &VectorVectorIntersectResult2D::get_param_t);
	ClassDB::bind_method(D_METHOD("get_param_u"), &VectorVectorIntersectResult2D::get_param_u);
	ClassDB::bind_method(D_METHOD("equalize", "other"), &VectorVectorIntersectResult2D::equalize);
}

/*
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

	if (t >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && t <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON &&
		u >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && u <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
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

	if (t >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && t <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON &&
			u >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && u <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) {
		return 1;
	}
	return 0;
}
*/

// Usage of LinePlaneIntersectResult3D makes this perfectly GDScript compatible
bool PolygonPhysicsSystem2D::solve_line_plane_collision_with_result(Vector3 line1_point, Vector3 line1_dir,
		Vector3 plane1_point, Vector3 plane1_dir_u, Vector3 plane1_dir_v, LinePlaneIntersectResult3D *result_out) {

	if (unlikely(result_out == nullptr)) {
		print_error("result_out doesn't exist at PolygonPhysicsSystem2D::solve_line_plane_collision_with_result! [GDScript] Did you call new()?");
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

	result_out->was_processed = true;

	return ((result_out->param_t >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result_out->param_t <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) &&
			(result_out->param_u >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result_out->param_u <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) &&
			(result_out->param_v >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result_out->param_v <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON));
}

bool PolygonPhysicsSystem2D::solve_vector_vector_collision_with_result(Vector2 vec1_point, Vector2 vec1_dir,
		Vector2 vec2_point, Vector2 vec2_dir, VectorVectorIntersectResult2D *result_out) {

	if (unlikely(result_out == nullptr)) {
		print_error("result_out doesn't exist at PolygonPhysicsSystem2D::solve_vector_vector_collision_with_result! [GDScript] Did you call new()?");
		return false;
	}

	real_t denominator = vec1_dir.x * vec2_dir.y - vec2_dir.x * vec1_dir.y;

	result_out->param_t = (vec2_dir.y * vec2_point.x - vec2_dir.y * vec1_point.x - vec2_dir.x * vec2_point.y + vec2_dir.x * vec1_point.y) / denominator;
	result_out->param_u = -(-vec1_dir.y * vec2_point.x + vec1_dir.y * vec1_point.x + vec1_dir.x * vec2_point.y - vec1_dir.x * vec1_point.y) / denominator;

	return ((result_out->param_t >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result_out->param_t <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON) &&
			(result_out->param_u >= 0 - MAXQ_REAL_T_ARBITRARY_EPSILON && result_out->param_u <= 1 + MAXQ_REAL_T_ARBITRARY_EPSILON));
}

void PolygonPhysicsSystem2D::_bind_methods() {
	/*
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_with_result", "line1_point1", "line1_point2", "line1_offset",
					"line2_point1", "line2_point2", "line2_offset"), &PolygonPhysicsSystem2D::solve_line_intersect_with_result_GDS);
	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
		D_METHOD("solve_line_intersect_simple", "line1_point1", "line1_point2", "line1_offset",
					"line2_point1", "line2_point2", "line2_offset"),
			&PolygonPhysicsSystem2D::solve_line_intersect_simple);
	*/

	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
			D_METHOD("solve_vector_vector_collision_with_result", "vec1_point", "vec1_dir",
					"vec2_point", "vec2_dir", "result_out"),
			&PolygonPhysicsSystem2D::solve_vector_vector_collision_with_result);

	ClassDB::bind_static_method("PolygonPhysicsSystem2D",
			D_METHOD("solve_line_plane_collision_with_result", "line1_point1", "line1_dir_vec",
					"plane1_point1", "plane1_dir_u", "plane1_dir_v", "result_out"),
			&PolygonPhysicsSystem2D::solve_line_plane_collision_with_result);
}
