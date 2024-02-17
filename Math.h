#pragma once

#include <corecrt_math_defines.h>

#include "include.h"

const float INF = FLT_MAX;
const float kPi = (float)M_PI;

//二次元ベクトル
struct Vec2 {
public:
	float x_ = 0;
	float y_ = 0;

	Vec2();
	Vec2(const float x,const float y);

	void set(const float, const float);
	
	//演算
	float dot(const Vec2)const;//内積
	float cross(const Vec2)const;//外積
	float norm()const;//大きさ
	float normSquared()const;
	float distance(const Vec2)const;//２点間の距離
	Vec2 normalize()const;//正規化
	Vec2 rotationCCW(const float &)const;//回転
	Vec2 normalCCW()const;

	//演算子オーバーロード
	Vec2 operator+(const Vec2)const;
	Vec2 operator+(const float)const;
	Vec2 operator-(const Vec2)const;
	Vec2 operator-(const float)const;
	Vec2 operator*(const float)const;
	Vec2 operator/(const float)const;

	std::string toString()const;
};

struct Matrix {
	int row_num_;//列
	int column_num_;//行
	std::vector<std::vector<float>> matrix_;//[row][column]
	
	Matrix(const int , const int);
	Matrix(const Vec2&);
	
	//行列演算
	void identity();//単位行列にする
	Matrix product(const Matrix&) const;//行列積
	Matrix trans()const;//転置行列を返す
	
	//演算子オーバーロード
	Matrix operator+(const float)const;
	Matrix operator+(const Matrix&)const;
	Matrix operator*(const float)const;
	Matrix operator*(const Matrix&)const;
	Matrix operator-(const Matrix&)const;

	std::string toString()const;
};

//線分
struct Segment {
public:
	Segment(){}
	Segment(const Vec2 , const Vec2);

	Vec2 start_;
	Vec2 end_;
};

//ラジアンと度数法の変換
float RadToDeg(const float);
float DegToRad(const float);

//点と直線の距離を求める
int posRelationSegPoint(const Vec2& , const Segment &);
float distanceLinePoint(const Vec2& , const Segment &);
float distanceSegPoint(const Vec2&, const Segment&);

//なす角を取得
float calAngle(const Vec2 & , const Vec2 & , const Vec2 &);

Vec2 LtoW(const Vec2& local,const Vec2& worldCen,const float ang);
Vec2 WtoL(const Vec2&, const Vec2& ,const float);

//円運動の速度を取得する v = rω
Vec2 rotationVelocity(const Vec2& ,const float);

//値をクランプする
float clamp(const float , const float , const float);