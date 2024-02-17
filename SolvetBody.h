#pragma once

#include "include.h"
#include "Math.h"
#include "Object.h"

//拘束を解くのに必要な情報

struct SolverBody{
public:
	SolverBody() {};
	SolverBody(const Object*);
	SolverBody(const float , const float , const float);
	Vec2 delta_linear_vel_ = Vec2();//並進運動の更新される速度
	float delta_rota_vel_ = 0.f;//回転運動の更新される速度
	float angle_;//姿勢
	float inertia_inv_;//慣性テンソルの逆数
	float mass_inv_;//質量の逆数

	std::string toString()const;
};