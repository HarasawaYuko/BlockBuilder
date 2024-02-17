#pragma once
#include "Math.h"
#include "Constraint.h"

//衝突点の情報を保持する構造体
struct ContactPoint {
	float depth_;
	Vec2 pointA_;//Aの衝突点ローカル座標
	Vec2 pointB_;//Bの衝突点ローカル座標
	Vec2 pointA_edge_;//Aの衝突点ローカル座標（辺上）
	Vec2 pointB_edge_;//Bの衝突点ローカル座標（辺上）
	Vec2 normal_vec_;//衝突法線ベクトル
	Constraint constraint_[2];//拘束 反発方向0 摩擦方向1
};