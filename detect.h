#pragma once
#include "Object.h"
#include "Convex.h"
#include "Circle.h"

namespace Detect {
	//ブロードフェーズ
	bool broard(const Object* ,const Object*);

	//ナローフェーズ 図形によってオーバーロード
	bool narrow(Convex* , Convex* , float* , Vec2* , Vec2* , Vec2* );
	bool narrow(Circle* , Convex* , float* , Vec2* , Vec2* , Vec2* );
	bool narrow(Circle* , Circle* , float* , Vec2* , Vec2* , Vec2* );

}