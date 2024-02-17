#include "Circle.h"
#include "Constant.h"

Circle::Circle(const float cen_x, const float cen_y, const float r, const float v_x, const float v_y , const bool act) 
	:r_(r) , Object(Vec2(cen_x , cen_y) , Vec2(v_x , v_y), CIRCLE, ( r * r * kPi), Constant::FRICTION ,COLOR_RED, act)
{
	makeBbox();
	inertiatensor_ = 0.5f * mass_ * r * r;
}

Circle::Circle(const Vec2 cen, const float r, const Vec2 v, const bool act)
	:r_(r) , Object(cen , v , CIRCLE, ( r * r * kPi), Constant::FRICTION , COLOR_RED, act)
{
	makeBbox();
	inertiatensor_ = 0.5f * mass_ * r * r;
}

void Circle::Draw() const {
	DrawCircleP(center_.x_, center_.y_, r_, color_, true);
}

void Circle::Draw(const int x_scroll, const int y_scroll) const {
	DrawCircleP(center_.x_ + x_scroll, center_.y_ + y_scroll, r_, color_, true);
	DrawCircleP(center_.x_ + x_scroll, center_.y_ + y_scroll, r_, COLOR_BLACK, false, 2.f);
}

void Circle::DrawEdge()const {
	DrawCircleP(center_.x_, center_.y_, r_, color_, false, 3.f);
}

void Circle::updatePos(const float timestep) {
	//動かない物体の場合何もしない
	if (!is_active_) {
		return;
	}
	Vec2 deltaLinearVelocity = (velocity_ * timestep);
	float deltaRotaVelocity = (angle_velocity_rad_ * timestep);
	if (deltaLinearVelocity.norm() > Constant::STOP_SPEED) {
		center_ = center_ + deltaLinearVelocity;
	}
	if (abs(deltaRotaVelocity) > Constant::STOP_SPEED) {
		angle_rad_ = angle_rad_ + deltaRotaVelocity;
	}
	//バウンディングボックスを設定
	makeBbox();
}

bool Circle::isValid(const int x, const int y)const {
	//完全に画面外に出たら
	const float upper = center_.y_ + r_;
	const float right = center_.x_ + r_;
	const float left  = center_.x_ - r_;
	if ( upper < 0.f || right < 0.f || left > x) {
		return false;
	}
	return true;
}

//縮尺を面積がareaになるように変更する
void Circle::changeSize(const float area_goal) {
	float nowArea = r_ * r_ * kPi;
	float rate = area_goal / nowArea;
	//相似比を使って半径を修正
	r_ *=  sqrtf(rate);
	//質量と慣性テンソルを再計算
	mass_ = r_ * r_ * kPi;
	inertiatensor_ = 0.5f * mass_ * r_ * r_;
}

void Circle::rotation(const float) {
	return;
}

void Circle::move(const Vec2 vec) {
	center_ = center_ + vec;
	makeBbox();
}

void Circle::makeBbox() {
	bbox_.origin_.x_ = center_.x_ - r_;
	bbox_.origin_.y_ = center_.y_ - r_;
	bbox_.height_ = r_ * 2.f;
	bbox_.width_ = r_ * 2.f;
}

float Circle::getR() const{
	return r_;
}

void Circle::setR(const float r) {
	r_ = r;
}