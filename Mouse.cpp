#include "Mouse.h"

//シングルトンへのアクセスを返す
Mouse* Mouse::instance() {
	static Mouse instance;
	return &instance;
}

//マウス情報の更新
void Mouse::update() {
	//マウス位置の取得
	GetMousePoint(&pos_x_ , &pos_y_);
	pos_y_ = WIN_SIZE_Y - pos_y_;
	wheel_ = GetMouseWheelRotVol();
	int nowInput = GetMouseInput();
	for (int i = 0; i < INPUT_NUM; i++) {
		if ((nowInput >> i) & 1) {
			mous_iInput_[i]++;
		}
		else {
			mous_iInput_[i] = 0;
		}
	}
}

//クリックされているかを取得
bool Mouse::isClick(clickCode code) {
	return mous_iInput_[code] > 0;
}

//今クリックされた瞬間かを取得
bool Mouse::isClickNow(clickCode code) {
	return mous_iInput_[code] == 1;
}

float Mouse::getX() {
	return (float)pos_x_;
}

float Mouse::getY() {
	return (float)pos_y_;
}

int Mouse::getWheel() {
	return wheel_;
}