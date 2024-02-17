#pragma once
#include "include.h"

//マウスを管理するクラス（シングルトン）

constexpr int INPUT_NUM = 2;

//アクセス用コード
enum clickCode {
	LEFT_CLICK = 0,
	RIGHT_CLICK = 1,
};

class Mouse {
private:
	//位置
	int pos_x_ = 0;
	int pos_y_ = 0;
	int wheel_ = 0;
	//クリック状態
	int mous_iInput_[INPUT_NUM] = {};
	//コンストラクタ、代入演算子をprivateに
	Mouse() {};
	Mouse(const Mouse&) {}
	Mouse& operator=(const Mouse&){}

public:
	static Mouse* instance();
	void update();

	bool isClick(clickCode);
	bool isClickNow(clickCode);
	float getX();
	float getY();
	int getWheel();
};