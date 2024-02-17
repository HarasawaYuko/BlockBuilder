#include "BBox.h"
#include "UIMaterial.h"

void BBox::Draw() const{
	const Vec2 upper_left  = origin_ + Vec2(0.f   , height_);
	const Vec2 lower_right = origin_ + Vec2(width_ ,    0.f);
	const Vec2 upper_right = origin_ + Vec2(width_ , height_);
	DrawSegment(origin_      , upper_left  , COLOR_BLACK);
	DrawSegment(origin_      , lower_right , COLOR_BLACK);
	DrawSegment(upper_left   , upper_right , COLOR_BLACK);
	DrawSegment(upper_right  , lower_right , COLOR_BLACK);
}