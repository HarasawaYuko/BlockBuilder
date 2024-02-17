#include "ObjectTest.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "detect.h"
#include "Convex.h"
#include "DEBUG.h"

//図形テスト
static int mode = 0;

//凸包テスト mode = 0
static std::vector<Vec2> points;
static Convex* convex;
static bool make = false;

//凸包回転
static bool rotation = false;
static Vec2 vel;
static float angV = kPi/2;
static float dot;

ObjectTest::ObjectTest(SceneChanger* changer)
	:BaseScene(changer)
{}

ObjectTest::~ObjectTest()
{}

void ObjectTest::Initialize() {
}

void ObjectTest::Update() {
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
		m_sceneChanger->ChangeScene(Scene_Result);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Detect);
	}

	Rand* rand = Rand::instance();
	switch (mode) {
	case 0:
		//凸包の作成
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_A)) {
			delete convex;
			make = false;
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			delete convex;
			convex = new Convex(points);
			convex->setColor(GetColor(rand->getI(0, 255), rand->getI(0, 255), rand->getI(0, 255)));
			points.clear();
			make = true;
		}
		break;
	case 1:
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_L)) {
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_A)) {
			delete convex;
			make = false;
			rotation = false;
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			delete convex;
			convex = new Convex(points,0,0,0,0,true);
			convex->setColor(GetColor(rand->getI(0, 255), rand->getI(0, 255), rand->getI(0, 255)));
			points.clear();
			((Object*)convex)->setAngVRad(angV);
			make = true;
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_R)) {
			rotation = !rotation;
		}
		if (rotation) {
			convex->updatePos(0.1f);
			vel = convex->rotationVelocityVec(convex->getPointL(0));
			dot = (convex->getPointW(0)-convex->getCenter()).dot(vel);
		}
		break;
	}
}

void ObjectTest::Draw() {
	SetFontSize(20);
	DrawString(640, 0, "ObjectTest", COLOR_BLACK);
	DrawString(600 , 22 ,"P->modeを進める M->modeを戻す" , COLOR_BLACK);
	switch (mode) {
	case 0:
		DrawString(0, 0, "凸包と凸包の生成\nLeftClick->point追加 L->pointClear A->all clear enter->make \nConvex D->detect\n", COLOR_BLACK);
		//point
		for (auto p : points) {
			DrawPoint(p , COLOR_RED);
		}
		//凸包の描画
		if (make) {
			convex->DrawEdge();
			DrawFormatString(0, 80, COLOR_RED, "慣性テンソル:%f", convex->getIne());
			DrawFormatString(0, 110, COLOR_RED, "質量:%f", convex->getMass());
		}
		break;
	case 1:
		DrawString(0, 0, "凸包の回転\nLeftClick->point追加 L->pointClear A->all clear enter->make \nConvex R->回転\n", COLOR_BLACK);
		//point
		for (auto p : points) {
			DrawPoint(p, COLOR_RED);
		}
		//凸包の描画
		if (make) {
			convex->DrawEdge();
			DrawFormatString(0, 80, COLOR_RED, "角速度 rad:%f fre:%f", convex->getAngVelRad() , DegToRad(convex->getAngVelRad()));
			DrawFormatString(0, 110, COLOR_RED, "内積 %f", dot);
			DrawFormatString(0, 140, COLOR_BLACK, "大きさ %f", vel.norm());
			//printfDx("%f \n", convex->getPointL(0).toString().c_str());
			//printfDx("vel %s\n" , vel.toString().c_str());
			//速度ベクトルの描画
			Segment vSeg = Segment(convex->getPointW(0) , (convex->getPointW(0)+vel));
			DrawSegment(vSeg, COLOR_RED);
			DrawSegment(Segment(convex->getCenter() , convex->getPointW(0)), COLOR_BLACK);
		}
		break;
	}

}

void ObjectTest::Finalize() {
}

//画像、音声のメモリ解放
void ObjectTest::deleteMem() {
}