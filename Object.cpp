#include "Object.h"
#include "Constant.h"

//コンストラクタ
Object::Object(Vec2 cen , Vec2 v, Type type, float mass,float fri , Color color, bool act, float ang, float ang_v)
	:center_(cen) , velocity_(v), type_(type), color_(color), mass_(mass),friction_(fri), is_active_(act), angle_rad_(ang), angle_velocity_rad_(ang_v), e_(Constant::RESTITUTION)
{}

//移動、変形
void Object::addVel(const Vec2 acc) {
	if (is_active_) {
		velocity_ = acc + velocity_;
	}
}

void Object::addVelAng(const float acc) {
	if (is_active_){
		angle_velocity_rad_ += acc;
	}
}

//円運動の速度を取得　point ローカル座標
//return ワールド座標での速度
Vec2 Object::rotationVelocityVec(const Vec2& point_local ,const float omega )const {
	//ワールド座標に戻して返す
	return rotationVelocity(point_local , omega).rotationCCW(angle_rad_);
}

Vec2 Object::rotationVelocityVec(const Vec2& point_local)const {
	//ワールド座標に戻して返す
	return rotationVelocity(point_local, angle_velocity_rad_).rotationCCW(angle_rad_);
}

//ゲッター
const Vec2& Object::getCenter()const {
	return center_;
}

const Vec2& Object::getVelocity()const {
	return velocity_;
}

const float& Object::getMass() const{
	return mass_;
}

const float& Object::getAngleRad()const {
	return angle_rad_;
}

const float& Object::getAngVelRad()const {
	return angle_velocity_rad_;
}

const float& Object::getIne()const {
	return inertiatensor_;
}

const float& Object::getFri()const {
	return friction_;
}

const float& Object::getE()const {
	return e_;
}

const Type& Object::getType()const {
	return type_;
}

const uint16_t& Object::getTotalId() const {
	return id_total_;
}

const int& Object::getIndex()const {
	return index_;
}

const BBox& Object::getBbox()const {
	return bbox_;
}

const bool& Object::isActive()const {
	return is_active_;
}

bool Object::isTouch()const {
	return is_touch_;
}

const Color& Object::getColor() const {
	return color_;
}

//セッター
void Object::setC(const Vec2 c) {
	center_ = c;
}

void Object::setV(const Vec2 v) {
	velocity_ = v;
}

void Object::setMass(const float m) {
	mass_ = m;
}

void Object::setAngleRad(const float ang) {
	angle_rad_ = ang;
}

void Object::setAngVRad(const float ang_v) {
	angle_velocity_rad_ = ang_v;
}

void Object::setFri(const float fri) {
	friction_ = fri;
}

void Object::setE(const float e) {
	e_ = e;
}

void Object::setTotalId(const uint16_t i) {
	id_total_ = i;
}

void Object::setIndex(const int i) {
	index_ = i;
}

void Object::setTouch(const bool touch) {
	is_touch_ = touch;
}

void Object::setColor(const Color c) {
	this->color_ = c;
}

std::string Object::toString()const {
	return "Object";
}