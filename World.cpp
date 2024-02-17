#include "Pair.h"
#include "detect.h"
#include "World.h"
#include "Solver.h"

//物理シミュレーション本体

World::World(float timeStep , const int x , const int y)
	:WORLD_SIZE_X(x) , WORLD_SIZE_Y(y) , TIME_STEP(timeStep)
{}

void World::initialize() {
	//初期化
	obj_num_ = 0;
	pairs_.clear();
	objects_.clear();
}

void World::physicsSimulate() {
	//外力を加える
	applyGravity();

	//衝突検知
	detectCollision();
	
	//拘束の解消
	solveConstraints();
	
	//位置の更新
	updatePosition();
}

uint16_t World::addObj(Object* obj) {
	objects_.push_back(obj);
	obj_num_++;
	total_obj_num_++;
	obj->setTotalId(total_obj_num_);

	//オブジェクト種類によってソート
	sortObj();
	//インデックスの振り直し
	for (int i = 0; i < objects_.size(); i++) {
		objects_[i]->setIndex(i);
	}

	return obj->getTotalId();
}

//力を加える
void World::applyGravity() {
	//重力加速度を計算
	float acc = -GRAVITY * TIME_STEP;
	for (Object* obj : objects_) {
		obj->addVel(Vec2(0, acc));
	}
}

//衝突検知
void World::detectCollision() {
	//ブロードフェーズ
	detectBroard();

	//ナローフェーズ
	for (auto it = pairs_.begin(); it != pairs_.end();) {
		const Pair& pair = *it;
		float depth;//貫通深度
		Vec2 n_vec;//衝突法線ベクトル
		Vec2 contact_coord_same[2];//衝突座標
		Vec2 contact_coord_edge[2];//衝突座標（辺上）
		//衝突した種類で場合分け
		switch (pair.getCombi()) {
		case CONVEX_CONVEX: {
			//ダウンキャスト
			Convex* con1 = static_cast<Convex*>(pair.getObj0());
			Convex* con2 = static_cast<Convex*>(pair.getObj1());
			if (Detect::narrow(con1, con2, &depth, &n_vec, contact_coord_same, contact_coord_edge)) {
				it++;
			}
			else {
				it = pairs_.erase(it);
				continue;
			}
		}
			break;
		case CIRCLE_CIRCLE: {
			//ダウンキャスト
			Circle* cir1 = static_cast<Circle*>(pair.getObj0());
			Circle* cir2 = static_cast<Circle*>(pair.getObj1());
			if (Detect::narrow(cir1, cir2, &depth, &n_vec, contact_coord_same, contact_coord_edge))
				it++;
			else {
				it = pairs_.erase(it);
				continue;
			}
		}
			break;
		case CIRCLE_CONVEX: {
			//ダウンキャスト
			Circle* cir = static_cast<Circle*>(pair.getObj0());
			Convex* con = static_cast<Convex*>(pair.getObj1());
			if (Detect::narrow(cir, con, &depth, &n_vec, contact_coord_same, contact_coord_edge))
				it++;
			else {
				it = pairs_.erase(it);
				continue;
			}
			break;
		}
		}
		//衝突していたら衝突情報を記録
		ContactPoint cp;
		cp.depth_ = depth;
		cp.normal_vec_ = n_vec;
		cp.pointA_ = contact_coord_same[0];
		cp.pointB_ = contact_coord_same[1];
		cp.pointA_edge_ = contact_coord_edge[0];
		cp.pointB_edge_ = contact_coord_edge[1];

		pair.getCol()->addCp(cp);
		pair.getObj0()->setTouch(true);
		pair.getObj1()->setTouch(true);
	}
}


//拘束の解消
void World::solveConstraints() {
	//全てのペアについて
	Solver::solve(this);
}

//位置の更新
void World::updatePosition() {
	auto obj = objects_.begin();
	bool isErase = false;//オブジェクトが削除されたか
	while (obj != objects_.end())
	{
		(*obj)->updatePos(TIME_STEP);
 		if ((*obj)->isValid(WORLD_SIZE_X , WORLD_SIZE_Y))
		{
			obj++;
		}
		else
		{
			obj = objects_.erase(obj);
			isErase = true;
			obj_num_--;
		}
	}
	//削除があったら
	if (isErase) {
		//インデックスの振り直し
		for (int i = 0; i < objects_.size(); i++) {
			objects_[i]->setIndex(i);
		}
	}
}

//描画　scroll 分ずらして描画
void World::Draw(const int x_scroll, const int y_scroll)const {
	for (auto& obj : objects_) {
		obj->Draw(x_scroll , y_scroll);
	}
}

Object* World::searchObjById(uint16_t id)const {
	for (auto& obj : objects_) {
		if (obj->getTotalId() == id) {
			return obj;
		}
	}
	return NULL;
}

void World::finalize() {
	for (auto& obj : objects_) {
		delete obj;
	}
	objects_.clear();
	pairs_.clear();
}

void World::sortObj() {
	std::sort(objects_.begin(), objects_.end(), [](const Object* a, const Object* b) {
		return (uint16_t)a->getType() < (uint16_t)b->getType();
		});
}

//ブロードフェーズ
void World::detectBroard() {
	std::vector<Pair> currentPairs;//検出されたペア
	for (int i = 0; i < obj_num_; i++) {
		for (int j = i + 1; j < obj_num_; j++) {
			//バウンディングボックスによる判定
			if (Detect::broard(objects_[i], objects_[j])) {
				//見つかったらペアを作成
				currentPairs.emplace_back(objects_[i], objects_[j]);
				currentPairs.back().setType(New);
			}
		}
	}
	//前のリストと比較してTypeを設定
	for (int cur_i = 0; cur_i < currentPairs.size(); cur_i++) {
		for (int pre_i = 0; pre_i < pairs_.size(); pre_i++) {
			//同じ種類が検出されたら
			if (currentPairs[cur_i].getKey() == pairs_[pre_i].getKey()) {
				currentPairs[cur_i] = pairs_[pre_i];
				currentPairs[cur_i].setType(Keep);
			}
		}
	}
	//ペアの配列を更新
	this->pairs_ = currentPairs;

	//以前の衝突点の状況を確認し、更新する
	for (Pair& pair : pairs_) {
		pair.checkContactPoints();
	}
}