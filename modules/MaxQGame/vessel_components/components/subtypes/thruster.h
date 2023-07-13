/* thruster.h */

#pragma once

#include "scene\2d\node_2d.h"
#include "scene/2d/physics_body_2d.h"

#include "../component.h"


class Thruster : public Component {
	GDCLASS(Thruster, Component);

protected:
	void _notification(int p_what) override;
	static void _bind_methods();

	void apply_thrust();

	void find_physics_body();
	void null_physics_body();

	static String physicsBodyTypeID;
	RigidBody2D *releveant_body = nullptr;

	Vector2 precomp_force = { 0, 0 };
	real_t precomp_torque = 0;

public:
	Thruster();
	void autodetermine_dir();
	void precompute_forces();

	int power = 0;
	bool ready = false; // Have to massively increase power the first time we're parented because of how I wrote this (I am not sorry)

	// TODO: expose all this to gdscript

	bool main_engine = false;

	bool autodetermine_direction = true;

	bool move_front = false;
	bool move_back = false;
	bool move_left = false;
	bool move_right = false;
	bool rotate_left = false;
	bool rotate_right = false;

	bool apply_thrust_next_tick = false;

	// Setters and getters for the above, needed for exposing variables.

	void set_power(int i) {
		power = i;
	};
	int get_power() {
		return power;
	};

	void set_main_engine(bool b){
		main_engine = b;
	};
	void set_move_front(bool b) {
		move_front = b;
	};
	void set_move_back(bool b) {
		move_back = b;
	};
	void set_move_left(bool b) {
		move_left = b;
	};
	void set_move_right(bool b) {
		move_right = b;
	};
	void set_rotate_left(bool b) {
		rotate_left = b;
	};
	void set_rotate_right(bool b) {
		rotate_right = b;
	};

	bool get_main_engine() {
		return main_engine;
	};
	bool get_move_front() {
		return move_front;
	};
	bool get_move_back() {
		return move_back;
	};
	bool get_move_left() {
		return move_left;
	};
	bool get_move_right() {
		return move_right;
	};
	bool get_rotate_left() {
		return rotate_left;
	};
	bool get_rotate_right() {
		return rotate_right;
	};
};
