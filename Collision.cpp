#include "Collision.h"

Collision::Collision() 
	:contact_num_(0)
{}

Collision::Collision(Object* obj1 , Object* obj2 )
	:contact_num_(0) , obj1_(obj1) , obj2_(obj2)
{}

//もっと良い実装方法がありそう
void Collision::addCp(ContactPoint add_point) {
	//既存の衝突点と同じ点であれば、それを更新する
	const float kSamePoint = Constant::JUDGE_SAME_POINT;//同じ点と見なせる範囲
	int exist_point_index = -1;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint& exist_point = contactpoints_[i];
		//衝突新規の点と比較
		const float distance_A = (add_point.pointA_ - exist_point.pointA_).norm();
		const float distance_B = (add_point.pointB_ - exist_point.pointB_).norm();
		const float dot_nvec = add_point.normal_vec_.dot(exist_point.normal_vec_);
		if (distance_A < kSamePoint && distance_B < kSamePoint && dot_nvec > 0.99f ) {
			exist_point_index = i;
			break;
		}
	}

	//既存の衝突点と同一点が検出されたら
	if (exist_point_index != -1) {
		//既存の衝突点を新しい情報に更新する
		contactpoints_[exist_point_index].depth_ = add_point.depth_;
		contactpoints_[exist_point_index].normal_vec_ = add_point.normal_vec_;
		contactpoints_[exist_point_index].pointA_ = add_point.pointA_;
		contactpoints_[exist_point_index].pointB_ = add_point.pointB_;
		return;
	}

	//新規衝突点の場合
	//まだ追加できる場合
	if (contact_num_ < 2) {
		contactpoints_.push_back(add_point);
		contact_num_++;
		return;
	}

	//既に二つある場合
	contactpoints_.push_back(add_point);
	 contact_num_++;
	//貫通深度が最も深いものは必ず採用する
	int left_points_indexes[2];//採用する衝突点のインデックス
	ContactPoint deepest_point;
	float deep_max = FLT_MAX;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint cp = contactpoints_[i];
		if ( cp.depth_ < deep_max ) {
			deep_max = cp.depth_;
			deepest_point = cp;
			left_points_indexes[0] = i;
		}
	}

	//確定した点から最も遠い点を選択する
	float distace_max = FLT_MIN;
	for (int i = 0; i < contact_num_; i++) {
		const ContactPoint cp = contactpoints_[i];
		const float distance_with_deepest = cp.pointA_.distance(deepest_point.pointA_);
		if (distance_with_deepest > distace_max) {
			distace_max = distance_with_deepest;
			left_points_indexes[1] = i;
		}
	}

	for (int i = 0; i < contact_num_;i++) {
		if (i == left_points_indexes[0] || i == left_points_indexes[1]) {
			continue;
		}
		else {
			contactpoints_.erase(contactpoints_.begin() + i);
			contact_num_--;
			break;
		}
	}
	return;
}

void Collision::deleteCp(const int index) {
	assert(0 <= index && index < contact_num_);
	contactpoints_.erase(contactpoints_.begin() + index);
	contact_num_--;
}

Object* Collision::getObj1()const {
	return obj1_;
}

Object* Collision::getObj2()const {
	return obj2_;
}

float Collision::getE()const {
	return e_;
}

float Collision::getFri()const {
	return friction_;
}

int Collision::getContactNum()const {
	return contact_num_;
}

ContactPoint& Collision::getCp(const int i) {
	return contactpoints_[i];
}

void Collision::setFri(const float fri) {
	friction_ = fri;
}

void Collision::setE(const float e) {
	e_ = e;
}