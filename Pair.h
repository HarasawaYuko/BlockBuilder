#pragma once
#include "Math.h"
#include "Object.h"
#include "Collision.h"

/*
*ペアクラス
* ブロードフェーズで見つかった組み合わせを保存しておく
*/

//衝突物体の組み合わせ
enum Combi : uint8_t {
	CIRCLE_CIRCLE = 0b0000'0001,
	CIRCLE_CONVEX = 0b0000'0011,
	CONVEX_CONVEX = 0b0000'0010
};

//発見のタイミング
//New 今のタイムステップで検知された
//Keep 前のタイムステップで検知されている
enum PairType {
	New,
	Keep
};

class Pair {
private:
	Object* objects_[2];//オブジェクトへのアクセス
	Combi combi_kind_;//オブジェクトの組み合わせ
	PairType type_;//新規に見つかったペアか
	uint32_t key_;//ユニークなキー
	Collision* collision_;//衝突情報

public:
	Pair( Object* , Object*);

	//シミュレーション
	void checkContactPoints();

	//ゲッター
	const Combi& getCombi()const;
	const PairType& getType()const;
	uint32_t getKey()const;
	Object* getObj0()const;
	Object* getObj1()const;
	Collision* getCol()const;
	//セッター
	void setType(PairType);
	
	std::string toString()const;
};