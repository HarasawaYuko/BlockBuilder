#pragma once
#include "include.h"

//KeyBoardクラス(シングルトン)

class KeyBoard {
private:
	int key_state_[256];

	//コンストラクタ、コピーコンストラクタ、代入演算子をprivate
	KeyBoard(){};
	KeyBoard(const KeyBoard&) {};
	KeyBoard& operator=(const KeyBoard&) {};
public:
	static KeyBoard* instance();

	void update();

	bool isHitNow(const int);
	bool isPush(const int);
	int getFrame(const int);
};