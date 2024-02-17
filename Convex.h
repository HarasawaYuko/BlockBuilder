#pragma once

#include "Object.h"

//凸包クラス
class Convex :public Object {
private:
	std::vector<Vec2> points_local_;//ローカル座標
	std::vector<Vec2> points_world_;//ワールド座標
	int point_num_;

public:
	Convex() {}
	Convex(const std::vector<Vec2>& points , const float v_x = 0, const float v_y = 0, const float ang = 0, const float ang_v = 0, const bool act = false);
	void operator=(const Convex&);

	//シミュレーションを進める
	void updatePos(const float step)override;
	bool isValid(const int, const int)const override;

	//描画
	void Draw()const override;
	void Draw(const int x_scroll, const int y_scroll)const override;
	void DrawEdge()const override;

	//形状・位置の操作
	void move(const Vec2)override;
	void rotation(const float)override;
	void addAngle(const float);
	void changeSize(const float area) override;
	
	//ゲッター
	Vec2 getPointW(const int)const;
	Vec2 getPointL(const int)const;
	Segment getEdgeW(const int&)const;
	const std::vector<Vec2>& getPointsWorld();
	const std::vector<Vec2>& getPointsLocal();
	int getPointNum()const;

	//セッター
	void makeBbox()override;
	void setC(const Vec2)override;
	void recalWorldPoint();
};
