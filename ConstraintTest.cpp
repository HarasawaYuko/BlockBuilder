#include "ConstraintTest.h"
#include "Solver.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "detect.h"
#include "Convex.h"
#include "DEBUG.h"

//衝突検知テスト
static int mode = 0;

//Convex vs Convex mode =  0
static std::vector<Object*> convexes;
static std::vector<Vec2> points;
static std::vector<Collision> collisions;
static Vec2 contactP[2];
static bool move = false;
static bool detect = false;
static Vec2 gravity = Vec2(0.f, -5.f);
static Vec2 nVec;

ConstraintTest::ConstraintTest(SceneChanger* changer)
	:BaseScene(changer)
{}

ConstraintTest::~ConstraintTest()
{}

void ConstraintTest::Initialize() {
	//三角形二つを事前に作成
	points.emplace_back(200.f , 400);
	points.emplace_back(200.f, 550);
	points.emplace_back(350.f, 400);
	Convex* con = new Convex(points , 30 , 0 , 0 , 30.f , true);
	//con->setIndex(0);
	convexes.push_back(con);
	points.clear();
	
	//points.emplace_back(350.f, 450);
	//points.emplace_back(350.f, 550);
	//points.emplace_back(450.f, 450);
	//con = new Convex(points, -35, 0, 0, 30.f, true);
	//con->setIndex(1);
	//points.clear();
	//convexes.push_back(con);

	points.emplace_back(100.f, 300);
	points.emplace_back(100.f, 0);
	points.emplace_back(600.f, 300);
	points.emplace_back(600.f, 0);
	con = new Convex(points, 0, 0, 0, 0, false);
	con->setIndex(1);
	convexes.push_back(con);
	points.clear();
}

void ConstraintTest::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	if (KeyBoard::instance()->isHitNow(KEY_INPUT_P)) {
		mode++;
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_M)) {
		mode--;
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Math);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Whole);
	}
}

void ConstraintTest::Draw() {
	SetFontSize(20);
	DrawString(640, 0, "ConstraintTest", COLOR_BLACK);
	switch (mode) {
	case 0:
		Rand * rand = Rand::instance();
		DrawString(0, 25, "凸包と凸包の拘束解消\nLeftClick->point追加 L->pointClear A->all clear enter->make \nConvex D->detect\n", COLOR_BLACK);
		//凸包の描画
		for (auto con : convexes) {
			((Convex*)con)->DrawEdge();
		}
		//pointの描画
		for (auto p : points) {
			DrawPoint(p, COLOR_RED);
		}
		if (detect) {
			DrawString(700 , 400 , "DETECT"  , COLOR_RED);
		}
		//その他描画
		DrawFormatString(10, 550, COLOR_BLUE, "n:%s", nVec.toString().c_str());
		break;
	}
}

void ConstraintTest::Finalize() {
}

//画像、音声のメモリ解放
void ConstraintTest::deleteMem() {
}