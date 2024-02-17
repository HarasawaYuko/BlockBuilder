#pragma once
#include "include.h"

//共通変数を管理するstaticクラス

class Share {
private:
	Share();

public:
	static int score;
	static int loadPic;

	//プロトタイプ宣言
	static void initialize();
};