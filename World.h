#pragma once
#include <random>

#include "Object.h"
#include "Pair.h"
#include "Constant.h"

//物理シミュレーションクラス

class World {
private:
	const float GRAVITY = Constant::GRAVITY;//重力加速度
	const float TIME_STEP;
	//シミュレーションワールドのサイズ
	const int WORLD_SIZE_X;
	const int WORLD_SIZE_Y;

	uint16_t total_obj_num_;//通算で追加したオブジェクトの数　65536以上になったら一旦リセットする必要あり
	int obj_num_;//world内にあるオブジェクトの数

	void applyGravity();
	void detectCollision();
	void solveConstraints();
	void updatePosition();

	void detectBroard();

	void sortObj();
	
public:
	//コンストラクタ
	World(float timeStep = 1.f/(float)FPS , const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y);
	void initialize();
	void finalize();
	void physicsSimulate();
	uint16_t addObj(Object*);
	Object* searchObjById(uint16_t id)const;
	void Draw(const int, const int) const;

	std::vector<Object*> objects_;
	std::vector<Pair> pairs_;
};