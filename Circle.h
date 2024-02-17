#pragma once
#include "Object.h"

//円クラス
class Circle :public Object{
private:
	float r_;//半径
public:
	Circle(const float cen_x = 0 , const float cec_y = 0 , const float r = 10.0f , const float v_x = 0, const float v_y = 0 , const bool act = true);
	Circle(const Vec2 cen , const float r , const Vec2 v , const bool act = false);

	//描画
	void Draw()const override;
	void Draw(const int x_scroll, const int y_scroll)const;
	void DrawEdge()const override;

	//シミュレーション
	void updatePos(const float step)override;
	bool isValid(const int, const int)const override;

	//移動、変形
	void rotation(const float)override;
	void changeSize(const float area)override;
	void move(const Vec2)override;
	void makeBbox()override;

	//ゲッター
	float getR()const;
	
	//セッター
	void setR(const float);

	//std::string toString()const override;
};