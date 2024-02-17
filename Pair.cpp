#include "Pair.h"

uint32_t calKey(const uint16_t , const uint16_t);

Pair::Pair(Object* obj1 , Object* obj2) {
	key_ = calKey(obj1->getTotalId() , obj2->getTotalId());
	combi_kind_ = (Combi)(obj1->getType() | obj2->getType());
	objects_[0] = obj1;
	objects_[1] = obj2;
	collision_ = new Collision();
}

//contactPointが有効かを確認する
void Pair::checkContactPoints() {
	for (int i = 0; i < collision_->getContactNum();) {
		ContactPoint cp = collision_->getCp(i);
		//それぞれから見た接触点をワールド座標に変換
		Vec2 pointA = LtoW(cp.pointA_edge_, objects_[0]->getCenter(), objects_[0]->getAngleRad());
		Vec2 pointB = LtoW(cp.pointB_edge_, objects_[1]->getCenter(), objects_[1]->getAngleRad());
		Vec2 BtoA = (pointA - pointB).normalize();
		//貫通深度と逆向きの時
		if (cp.normal_vec_.dot(BtoA) > 0) {
			collision_->deleteCp(i);
		}
		else i++;
	}
}

const Combi& Pair::getCombi()const {
	return combi_kind_;
}

const PairType& Pair::getType()const {
	return type_;
}

uint32_t Pair::getKey()const {
	return key_;
}

Object* Pair::getObj0() const{
	return objects_[0];
}

Object* Pair::getObj1() const{
	return objects_[1];
}

Collision* Pair::getCol()const {
	return collision_;
}

void Pair::setType(PairType type_) {
	this->type_ = type_;
}

std::string Pair::toString()const {
	std::string str;
	//種類の取得
	switch (combi_kind_) {
	case CIRCLE_CIRCLE:
		str += "Cir vs Cir";
		break;
	case CIRCLE_CONVEX:
		str += "Cir vs Con";
		break;
	case CONVEX_CONVEX:
		str += "Con vs Con";
		break;
	}
	str += "  " + std::to_string(key_);
	switch (type_) {
	case New:
		str += " New";
		break;
	case Keep:
		str += " Keep";
		break;
	}
	str += "\n";
	
	str += "contactPoint:" + std::to_string(collision_->getContactNum()) + "\n";
	for (int i = 0; i < collision_->getContactNum();i++) {
		ContactPoint cp = collision_->getCp(i);
		char tmp[255];
		sprintf_s(tmp, "%6.1f", cp.depth_);
		str += std::to_string(i) + " depth:" + std::string(tmp) + " pointA:" + cp.pointA_.toString() + " pointB:" + cp.pointB_.toString() + " normal" + cp.normal_vec_.toString() + "\n";
	}

	return str;
}

uint32_t calKey(const uint16_t id1 , const uint16_t id2 ) {
	uint32_t id1_32 = (uint32_t)id1;
	uint32_t id2_32 = (uint32_t)id2;
	if (id1_32 < id2_32) {
		id2_32 = id2_32 << 16;
	}
	else {
		id1_32 = id1_32 << 16;
	}
	return (id1_32 | id2_32);
}