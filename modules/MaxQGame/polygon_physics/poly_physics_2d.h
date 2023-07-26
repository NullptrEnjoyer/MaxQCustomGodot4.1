/* poly_physics_2d.h */

#pragma once

#include "scene\2d\node_2d.h"

// Is parent of all physics thingamajigs, only really there for sorting stuff and some universal functions

class PolygonPhysicsSystem2D : public Node2D {
	GDCLASS(PolygonPhysicsSystem2D, Node2D);

public:
	//The most critical part of this physics system: the line collision formula - now available to everyone

	/// <param name="*intersect_point">, aka the point of colision. Gives inf if parallel, can be checked with isinf()</param>
	/// <returns>whether we have an intersect (true) or not (false).</returns>
	static bool solve_line_intersect_with_result(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset, Vector2* intersect_point);

	/// <summary>
	/// For GDScript compatibility. This is dumm, we can't send a reference or a pointer to Vector2 so we have to return it.
	/// Use simple to check if it actually intersects and then use this.
	/// </summary>
	static Vector2 solve_line_intersect_with_result_GDS(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
			Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset);

	/// <returns>whether we have an intersect (true) or not (false).</returns>
	static bool solve_line_intersect_simple(Vector2 line1_point1, Vector2 line1_point2, Vector2 line1_offset,
		Vector2 line2_point1, Vector2 line2_point2, Vector2 line2_offset);

	static void _bind_methods();
};

/*
func _ready():
    var p11:Vector2 = l1.get_point_position(0) + l1.global_position;
    var p12:Vector2 = l1.get_point_position(1) + l1.global_position;
    
    var p21:Vector2 = l2.get_point_position(0) + l2.global_position;
    var p22:Vector2 = l2.get_point_position(1) + l2.global_position;
    
    var a = p11.x
    var b = p11.y
    var c = p12.x - a
    var d = p12.y - b
    
    var e = p21.x
    var f = p21.y
    var g = p22.x - e
    var h = p22.y - f
    
    print("Vector 1 has starting point " + str(a) + ", " + str(b) + " with direction vector " + str(c) + ", " + str(d))
    print("Vector 2 has starting point " + str(e) + ", " + str(f) + " with direction vector " + str(g) + ", " + str(h))
    
    var resultt = (h*e - h*a - g*f + g*b) / (c*h - g*d)
    print("Result t is " + str(resultt))
    
    var resultu = -(-d*e + d*a + c*f - c*b) / (c*h - g*d)  ---> actually (-c*h + g*d), we can extract the - to only calculate the denominator once
    print("Result u is " + str(resultu))
*/
