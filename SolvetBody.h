#pragma once

#include "include.h"
#include "Math.h"
#include "Object.h"

//�S���������̂ɕK�v�ȏ��

struct SolverBody{
public:
	SolverBody() {};
	SolverBody(const Object*);
	SolverBody(const float , const float , const float);
	Vec2 delta_linear_vel_ = Vec2();//���i�^���̍X�V����鑬�x
	float delta_rota_vel_ = 0.f;//��]�^���̍X�V����鑬�x
	float angle_;//�p��
	float inertia_inv_;//�����e���\���̋t��
	float mass_inv_;//���ʂ̋t��

	std::string toString()const;
};