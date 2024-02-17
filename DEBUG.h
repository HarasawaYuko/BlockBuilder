#pragma once
#include "Math.h"
//デバッグ用の変数

class Debug {
private:
	//コンストラクタ、代入演算子をprivateに
	Debug() {};
	Debug(const Debug&) {}
	Debug& operator=(const Debug&) {}

public:
	static Debug* instance();
	//convex detect用
	Segment minEdgeB;
	Vec2 minPointA;
};