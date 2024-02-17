#include "detect.h"
#include "Circle.h"
#include "Convex.h"
#include "DEBUG.h"

//プロトタイプ宣言
void projection(const Vec2& ,const Convex* , float*, float*);
void getDepth(float* d1, float* d2, const Vec2& axis, const Convex* con1, const Convex* con2);
Vec2 calContactPointOnEdge(const Vec2& , const Segment&);
bool isCrossBbox(const BBox&, const BBox&);

//衝突点の取得
void computeContactCoord(Vec2*, Vec2* ,Convex* , Convex* , const float , const Vec2& );
void computeContactCoord(Vec2*, Vec2* ,const Circle* , const Convex* , const Vec2& , const float , const Vec2& );
void computeContactCoord(Vec2*, Vec2* ,Circle* , Circle* , const float , const Vec2& );
//分離軸判定
bool separationJudgeIsCross(const Vec2& , const Convex* , const Convex* , float& , Vec2& );

bool Detect::broard(const Object* obj1 ,const Object* obj2) {
	//バウンディングボックスを取得
	const BBox& bbox1 = obj1->getBbox();
	const BBox& bbox2 = obj2->getBbox();
	if (isCrossBbox(bbox1, bbox2)) {
		return true;
	}
	return false;
}

/**ナローフェーズ****************/
//凸包と凸包の衝突判定のオーバーロード
bool Detect::narrow(Convex* convex1 , Convex* convex2 , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//分離軸判定
	float depth_max = -INF;
	Vec2 axis_max_depth;//分離軸

	const std::vector<Vec2> points_world_con1 = convex1->getPointsWorld();
	const std::vector<Vec2> points_world_con2 = convex2->getPointsWorld();
	
	//頂点→頂点のベクトルの分離軸判定
	for (const auto& point1 : points_world_con1 ) {
		for (const auto& point2 : points_world_con2 ) {
			const Vec2 axis = (point1 - point2).normalize();
			//分離軸判定を行う
			if (!separationJudgeIsCross(axis , convex1 , convex2 , depth_max , axis_max_depth )) {
				return false;
			}
		}
	}
	//辺の法線ベクトルの分離軸判定
	//convex1
	for (int i = 0; i < convex1->getPointNum(); i++) {
		//軸(辺の法線ベクトル)を取得
		const Segment edge = convex1->getEdgeW(i);
		const Vec2 axis = (edge.start_ - edge.end_).normalize().normalCCW();
		
		//分離軸判定を行う
		if (!separationJudgeIsCross(axis, convex1, convex2, depth_max, axis_max_depth)) {
			return false;
		}
	}
	//convex2
	for (int i = 0; i < convex2->getPointNum(); i++) {
		//軸(辺の法線ベクトル)を取得
		const Segment edge = convex2->getEdgeW(i);
		const Vec2 axis = (edge.start_ - edge.end_).normalize().normalCCW();

		//分離軸判定を行う
		if (!separationJudgeIsCross(axis, convex1, convex2, depth_max, axis_max_depth)) {
			return false;
		}
	}

	//貫通深度等の設定
	*depth = depth_max;
	*n_vec = axis_max_depth;

	//衝突している場合は衝突点を取得する
	computeContactCoord( coord, coord_edge , convex1 , convex2 , depth_max , axis_max_depth );
	return true;
}

//円と凸包の衝突判定のオーバーロード
bool Detect::narrow(Circle* circle  , Convex* convex  , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//最短距離が半径より短ければ衝突とする
	const Vec2 center_cir = circle->getCenter();
	const float r = circle->getR();

	Vec2 point_min;
	float distance_min = FLT_MAX;

	//凸包の頂点→円までの距離
	for (int i = 0; i < convex->getPointNum(); i++) {
		const Vec2 point = convex->getPointW(i);
		const float distance = point.distance(center_cir);
		if (distance < distance_min) {
			distance_min = distance;
			point_min = point;
		}
	}
	//凸包の辺→円までの距離
	bool edge_has_minpoint = false;
	Segment edge_min;
	for (int i = 0; i < convex->getPointNum(); i++) {
		const Segment edge = convex->getEdgeW(i);
		const float distance= distanceSegPoint(center_cir , edge);
		if (distance < distance_min) {
			distance_min = distance;
			edge_min = edge;
			edge_has_minpoint = true;
		}
	}

	//最短距離で衝突判定
	if (distance_min > r) {
		return false;
	}

	//最近点を算出する
	if ( edge_has_minpoint ) {
		point_min = calContactPointOnEdge(center_cir , edge_min );
	}

	//貫通深度と法線ベクトルを取得
	*depth = distance_min - r;
	*n_vec = (center_cir - point_min).normalize();
	
	//衝突点を取得
	computeContactCoord(coord , coord_edge , circle , convex , point_min , *depth , *n_vec);

	return true;
}

//円と円の衝突判定のオーバーロード
bool Detect::narrow(Circle* circle1 , Circle* circle2 , float* depth , Vec2* n_vec , Vec2* coord , Vec2* coord_edge ) {
	//中心間の距離を取得
	const float distance_center = circle1->getCenter().distance(circle2->getCenter());
	const float r1 = circle1->getR();
	const float r2 = circle2->getR();
	if ((r1 + r2) < distance_center) {
		return false;
	}
	//貫通深度を取得
	*depth = distance_center - (r1 + r2);
	//法線ベクトル
	*n_vec = (circle1->getCenter() - circle2->getCenter()).normalize();

	computeContactCoord(coord, coord_edge, circle1, circle2, *depth, *n_vec);
	return true;
}

//衝突座標を取得
void computeContactCoord( Vec2* coord, Vec2* coord_edge ,Convex* convex1 , Convex* convex2 , const float depth , const Vec2& n_vec ) {
	float distance_min = INF;
	//頂点が接触点となる物体のインデックス
	//Aの物体の頂点なら0 ,Bの物体の頂点なら1
	int point_obj_index = 0;
	int point_min_index;
	int edge_min_index;


	//物体1を貫通深度より若干ずらす
	Vec2 move_vec = n_vec * (abs(depth) * 2.f);//ずらすベクトル
	convex1->move(move_vec);

	Convex* con_point = convex1;
	Convex* con_edge = convex2;
	for (int i = 0; i < 2; i++) {
		if (i == 1) {
			con_point = convex2;
			con_edge = convex1;
		}
		//最短距離を求める
		for (int it_p = 0; it_p < con_point->getPointNum(); it_p++) {
			for (int it_e = 0; it_e < con_edge->getPointNum(); it_e++) {
				const Vec2 point = con_point->getPointW(it_p);
				const Segment edge = con_edge->getEdgeW(it_e);
				const float dis = distanceSegPoint(point, edge);
				if (dis < distance_min) {
					//記録
					point_obj_index = i;
					distance_min = dis;
					point_min_index = it_p;
					edge_min_index = it_e;
					
				}
			}
		}
	}

	//ずらした分戻す
	convex1->move(move_vec * -1);

	if (point_obj_index == 0) {
		con_point = convex1;
		con_edge = convex2;
	}
	else {
		con_point = convex2;
		con_edge = convex1;
	}

	Vec2 min_point = con_point->getPointW(point_min_index);
	Segment min_edge = con_edge->getEdgeW(edge_min_index);

	coord[0] = WtoL(min_point , convex1->getCenter(), convex1->getAngleRad());
	coord[1] = WtoL(min_point , convex2->getCenter(), convex2->getAngleRad());
	coord_edge[point_obj_index] = coord[point_obj_index];
	const int edge_obj_index = (point_obj_index + 1) % 2;
	coord_edge[edge_obj_index] = WtoL(calContactPointOnEdge(min_point, min_edge), con_edge->getCenter(), con_edge->getAngleRad());
}

void computeContactCoord(Vec2* coord , Vec2* coord_edge , const Circle* circle  ,const Convex* convex , const Vec2& point_min , const float depth, const Vec2& vec_con_to_cir ) {
	//coord[0]は円上、coord[1]は凸包
	coord[0] = WtoL(point_min , circle->getCenter() , circle->getAngleRad());
	coord[1] = WtoL(point_min , convex->getCenter() , convex->getAngleRad());
	coord_edge[1] = coord[1];
	//円周上の衝突点を計算する
	//円の中心から円周上の点
	Vec2 contactpoint_circle = circle->getCenter() - (vec_con_to_cir * circle->getR());
	coord_edge[0] = WtoL(contactpoint_circle , circle->getCenter() , circle->getAngleRad());
}

void computeContactCoord(Vec2* coord, Vec2* coord_edge ,Circle* circle1 , Circle* circle2 , const float depth , const Vec2& vec_2to1) {
	//衝突点を取得 2の円周上
	const Vec2 center1 = circle1->getCenter();
	const Vec2 center2 = circle2->getCenter();
	Vec2 cir2_edge_contactpoint = center2 + (vec_2to1.normalize() * circle2->getR());
	coord[0] = WtoL(cir2_edge_contactpoint, center1 , circle1->getAngleRad());
	coord[1] = WtoL(cir2_edge_contactpoint, center2 , circle2->getAngleRad());
	coord_edge[0] = WtoL( center1 - ((vec_2to1 ) * circle1->getR()), center1 , circle1->getAngleRad());
	coord_edge[1] = coord[1];
}

/**************************************************************/

//分離軸判定を行う
//衝突しているかをreturnし、
//depth_maxと、axis_maxを設定する
bool separationJudgeIsCross(const Vec2& axis , const Convex* con1 , const Convex* con2 , float& depth_max , Vec2& axis_max ) {
	float depth1;
	float depth2;
	
	getDepth(&depth1, &depth2, axis, con1, con2);
	if (depth1 > 0.f || depth2 > 0.f) {
		return false;
	}

	if ( depth1 > depth_max ) {
		depth_max = depth1;
		axis_max = axis;
	}
	if ( depth2 > depth_max ) {
		depth_max = depth2;
		axis_max = axis * -1;
	}
	return true;
}

//axisにbox(convex)を投影して最大と最小を返す
void projection(const Vec2& axis, const Convex* con, float* min_result, float* max_result) {
	float min = INF;
	float max = -INF;
	//全ての頂点を投影
	for (int i = 0; i < con->getPointNum(); i++) {
		float dot = axis.dot(con->getPointW(i));
		min = min( min , dot);
		max = max( max , dot);
	}
	*min_result = min;
	*max_result = max;
}

void getDepth( float* d1 , float* d2 , const Vec2& axis , const Convex* con1 , const Convex* con2 ) {
	//投影座標の取得
	float max1, min1;
	float max2, min2;
	projection( axis , con1 , &min1 , &max1 );
	projection( axis , con2 , &min2 , &max2 );

	//貫通深度を取得
	*d1 = min1 - max2;
	*d2 = min2 - max1;
}

//軸投影の結果、分離軸があるか判定する　交差していない(d>0)の時はtrue
void getDepth(const float min1 , const float max1 ,const float min2 , const float max2  , float* d1 , float* d2) {

	//貫通深度を求めて、正負で判定する
	*d1 = min1 - max2;
	*d2 = min2 - max1;
}

Vec2 calContactPointOnEdge(const Vec2& point , const Segment& edge) {
	//辺ベクトルの正規化
	Vec2 edge_vec =(edge.end_ - edge.start_);
	Vec2 start_point_vec = point - edge.start_;
	//始点からの距離を求める
	float dis = start_point_vec.dot(edge_vec.normalize());
	dis = clamp(dis , 0.f , edge_vec.norm());
	return edge.start_ + (edge_vec.normalize() * dis);
}


bool isCrossBbox(const BBox& b1, const BBox& b2) {
	//x軸を調べる
	if (b1.origin_.x_ + b1.width_ <= b2.origin_.x_ || b1.origin_.x_ >= b2.origin_.x_ + b2.width_ ) {
		return false;
	}
	//y軸を調べる
	if (b1.origin_.y_ + b1.height_ <= b2.origin_.y_ ||b1.origin_.y_ >= b2.origin_.y_ + b2.height_ ) {
		return false;
	}
	return true;
}