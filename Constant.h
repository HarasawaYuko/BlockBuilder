#pragma once

//定数クラス

class Constant {
public:
	const static float BIAS;//めり込み解消用のバイアス
	const static float GRAVITY;//重力
	const static float STOP_SPEED;//停止と見なせる速度
	const static int FPS;
	//摩擦係数
	const static float FRICTION;
	//反発係数
	const static float RESTITUTION;
	const static float JUDGE_SAME_POINT;
	const static float INF;
};