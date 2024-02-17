#pragma once
#include "BBox.h"
#include "Constant.h"
#include "Math.h"
#include "UIMaterial.h"

/*
Objectクラス　演算対象の図形は必ず継承する
*/

//形状を表す列挙体
//組み合わせを表現するためにone hot表現をする
enum Type : uint8_t {
	CIRCLE = 0b0000'0001,
	CONVEX = 0b0000'0010
};

class Object {
protected:
	//運動、性質
	Vec2 center_;
	Vec2 velocity_;
	float mass_;
	float angle_rad_;//回転角　ラジアン
	float angle_velocity_rad_;//角速度　ラジアン
	float inertiatensor_;//慣性テンソル
	float friction_;//摩擦係数
	float e_;//反発係数

	//管理用
	Type type_;
	uint16_t id_total_;
	int index_;
	BBox bbox_;//バウンディングボックス（ワールド座標)
	bool is_touch_;

	//その他
	bool is_active_ = true;
	Color color_;

	Object(Vec2 cen ,Vec2 v , Type ,float mass = 10 , float fri = Constant::FRICTION , Color color = COLOR_BLACK ,bool act = true , float ang = 0.f , float ang_v = 0.f);

public:
	Object() {}

	//描画
	virtual void Draw()const = 0;
	virtual void Draw(const int scroll_x, const int scroll_y)const = 0;
	virtual void DrawEdge()const = 0;

	//シミュレーション
	virtual void updatePos(const float) = 0;
	virtual bool isValid(const int x = WIN_SIZE_X, const int y = WIN_SIZE_Y)const = 0;


	//移動、変形
	virtual void changeSize(const float area) = 0;
	virtual void move(const Vec2) = 0;
	virtual void rotation(const float) = 0;
	void addVel(const Vec2);
	void addVelAng(const float);
	virtual void makeBbox() = 0;

	Vec2 rotationVelocityVec(const Vec2& ,const float)const;
	Vec2 rotationVelocityVec(const Vec2&)const;

	//ゲッター
	const Vec2& getCenter()const;
	const Vec2& getVelocity()const;
	const float& getMass()const;
	const float& getAngleRad()const;
	const float& getAngVelRad()const;
	const float& getIne()const;
	const float& getFri() const;
	const float& getE()const;
	const Type& getType()const;
	const uint16_t& getTotalId()const;
	const int& getIndex()const;
	const BBox& getBbox()const;
	const bool& isActive()const;
	bool isTouch()const;
	const Color& getColor()const;

	//セッター
	virtual void setC(const Vec2);
	void setV(const Vec2);
	void setMass(const float);
	void setAngleRad(const float);
	void setAngVRad(const float);
	void setFri(const float);
	void setE(const float);
	void setTotalId(const uint16_t);
	void setIndex(const int);
	void setTouch(const bool);
	virtual void setColor(const Color color);

	std::string toString()const;//todo:virtualにした方が良い
};