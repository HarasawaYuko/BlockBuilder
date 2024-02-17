#pragma once
#include "include.h"
#include "Math.h"

//S‘©ŒvZ‚É•K—v‚ÈÕ“Ëî•ñ

struct Constraint {
	Vec2 axis_;//S‘©²
	float f_denominator_;//•ª•ê
	float f_;//‰ŠúS‘©—Í
	float lower_f_;
	float upper_f_;
	float accum_impulse_ = 0.f;//’~Ï‚³‚ê‚½S‘©—Í
};