/* poly_physics_2d.h */

#pragma once

#include "scene\2d\node_2d.h"
#include "core\object\ref_counted.h"

class LinePlaneIntersectResult3D : public RefCounted {
	GDCLASS(LinePlaneIntersectResult3D, RefCounted);

public:
	Vector3 intersect_point = {0, 0, 0};
	Vector3 intersect_normal = { 0, 0, 0 };

	real_t determinant = 0;
	real_t param_t = 0;
	real_t param_u = 0;
	real_t param_v = 0;
	bool was_processed = false;

	/// <returns>
	/// d is determinant, t is the line's direction vector multiplier, u and v are the plane's direction vector multipliers,
	/// therefore you're getting dtuv, in that order, in a Vector4
	/// </returns>
	Vector4 get_intersect_dtuv_data();
	Vector3 get_intersect_point();
	Vector3 get_intersect_normal();
	static void _bind_methods();
	void equalize(LinePlaneIntersectResult3D *other);
};

class VectorVectorIntersectResult2D : public RefCounted {
	GDCLASS(VectorVectorIntersectResult2D, RefCounted);

public:
	real_t param_t = 0;
	real_t param_u = 0;

	real_t get_param_t();
	real_t get_param_u();

	void equalize(VectorVectorIntersectResult2D *other);

	static void _bind_methods();
};

// Is parent of all physics thingamajigs, only really there for sorting stuff and some universal functions
class PolygonPhysicsSystem2D : public Node2D {
	GDCLASS(PolygonPhysicsSystem2D, Node2D);

public:

	/*
	//The most critical part of this physics system: the line collision formula - now available to everyone
	
	/// <param name="*intersect_point">, aka the point of colision. Gives inf if parallel, can be checked with isinf()</param>
	/// <returns>whether we have an intersect (true) or not (false).</returns>
	static bool solve_line_intersect_with_result(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset, Vector2* intersect_point);

	/// <summary>
	/// For GDScript compatibility. This is dumm, we can't send a reference or a pointer to Vector2 so we have to return it.
	/// Use simple to check if it actually intersects and then use this.
	/// Edit: Can make a class as godot passes them by reference (like in line-plane collisions), but it's fineeeeeeee
	/// </summary>
	static Vector2 solve_line_intersect_with_result_GDS(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
			Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset);

	/// <returns>whether we have an intersect (true) or not (false).</returns>
	static bool solve_line_intersect_simple(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset);

	*/

	static bool solve_line_plane_collision_with_result(Vector3 line1_point, Vector3 line1_dir,
			Vector3 plane1_point, Vector3 plane1_dir_u, Vector3 plane1_dir_v, LinePlaneIntersectResult3D *result_out);

	static bool solve_vector_vector_collision_with_result(Vector2 line1_point, Vector2 line1_dir,
			Vector2 line2_point, Vector2 line2_dir, VectorVectorIntersectResult2D *result_out);

	static void _bind_methods();
};
