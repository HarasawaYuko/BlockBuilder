#pragma once
#include "include.h"
#include "Math.h"

//バウンディングボックス

struct BBox {
	float height_;
	float width_;
	Vec2 origin_;//左下の点

	void Draw()const;
};