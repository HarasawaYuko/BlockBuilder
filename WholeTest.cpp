#include "WholeTest.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "Convex.h"

//プロトタイプ宣言
Convex* getCon(const int);

static bool move = false;
static Rand* rand_;
static std::vector<Vec2> points;
static float r;
static Vec2 cirCenter;
static Vec2 cirEdge;
static bool makeCon = true;

WholeTest::WholeTest(SceneChanger* changer)
	:BaseScene(changer)
{}

WholeTest::~WholeTest()
{}

void WholeTest::Initialize() {
	world.initialize();
	rand_ = Rand::instance();
	Convex* con = getCon(2);
	con->setColor(GetColor(rand_->getI(0, 155), rand_->getI(0, 155), rand_->getI(0, 155)));
	//床を設置
	world.addObj(con);

	//三角形を設置
	con = getCon(5);
	con->changeSize(4000.f);
	con->setColor(GetColor(rand_->getI(0, 155), rand_->getI(0, 155), rand_->getI(0, 155)));
	//con->setAngV(Pi/1.f);
	con->setV(Vec2(0 , 0));
	con->move(Vec2(400.f , 470.f));
	world.addObj(con);

	//三角形を設置
	con = getCon(7);
	con->setColor(GetColor(rand_->getI(0, 155), rand_->getI(0, 155), rand_->getI(0, 155)));
	//con->setAngV(Pi/1.f);
	con->setV(Vec2(0, 0));
	//con->move(Vec2(300, 50));
	world.addObj(con);
}

//円の並進運動
void WholeTest::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();
	//シーン切り替え
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_Game);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Detect);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_SPACE)) {
		move = !move;
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_R)) {
		Finalize();
		Initialize();
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_S)) {
		makeCon = !makeCon;
	}
	if (makeCon) {
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			//凸包の点を追加
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN) && points.size() >= 3) {
			Convex* con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
			con->setColor(GetColor(rand_->getI(0, 150), rand_->getI(0, 150), rand_->getI(0, 150)));
			world.addObj(con);
			points.clear();
		}
	}
	else {
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			cirCenter = Vec2(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (Mouse::instance()->isClickNow(RIGHT_CLICK)) {
			cirEdge = Vec2(Mouse::instance()->getX(), Mouse::instance()->getY());
			r = cirCenter.distance(cirEdge);
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN)) {
			Circle* cir = new Circle(cirCenter, r, Vec2(), true);
			cir->setColor(GetColor(rand_->getI(0, 150), rand_->getI(0 ,150), rand_->getI(0, 150)));
			world.addObj(cir);
			points.clear();
		}
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_A)) {
		points.clear();
	}
  	if(move)world.physicsSimulate();
	if (!move && Mouse::instance()->isClickNow(RIGHT_CLICK)) {
		world.physicsSimulate();
	}
}

void WholeTest::Draw() {
	SetFontSize(40);
	DrawString(0, 0, "WholeTest", COLOR_BLACK);
	SetFontSize(20);
	DrawString(640 , 23 , "LeftClick->point追加\nC->凸包追加\nA->point削除" , COLOR_BLACK);
	for (auto& obj : world.objects_) {
		obj->DrawEdge();
		//obj->getBbox().Draw();
		DrawPoint(obj->getCenter() ,COLOR_GREEN);
	}
	/*for (int i = 0; i < world.pairs.size();i++) {
		printfDx("ペア\n");
	}*/
	if (makeCon) {
		//pointの描画
		for (int i = 0; i < points.size(); i++) {
			DrawPoint(points[i], COLOR_RED);
		}
		DrawStrP(Vec2(600, 50), "凸包作成(Sで切り替え)", COLOR_BLUE);
	}
	else {
		DrawPoint(cirCenter , COLOR_RED);
		DrawPoint(cirEdge, COLOR_BLUE);
		DrawStrP(Vec2(600, 50), "円作成(Sで切り替え)", COLOR_BLUE);
	}
	//
/*その他描画*/
	//ペアの衝突情報
	//if (world.pairs.size()!= 0) {
	//	DrawString(0 , 20 ,"Detect!" , COLOR_RED);
	//	DrawFormatString(0 , 40 ,COLOR_BLUE, "%s" ,world.pairs[0].toString().c_str());
	//}
	//DrawFormatString(150 , 200 , COLOR_BLACK,"ペア数 %d\n" , world.pairs.size());
	////三角形の速度
	//if (world.objects.size() == 2) {
	//	DrawFormatString(world.objects[1]->getC().x + 40 , WIN_SIZE_Y - world.objects[1]->getC().y ,COLOR_GREEN , "%s" , world.objects[1]->getV().toString().c_str());
	//}
	////速度
	//for (int i = 0; i < world.objects.size(); i++) {
	//	DrawFormatString(600 , 30 + i*23, COLOR_BLUE , "V:%s\n" , world.objects[i]->getV().toString().c_str());
	//}
}

void WholeTest::Finalize() {
	for (int i = 0; i < world.objects_.size(); i++) {
		delete world.objects_[i];
	}
}

//画像、音声のメモリ解放
void WholeTest::deleteMem() {
}

//テスト用の図形取得
Convex* getCon(const int p) {
	std::vector<Vec2> points;
	Convex* con;
	switch (p) {
	case 0:
		//四角形
		points.emplace_back(30.f , 20.f);
		points.emplace_back(30.f , -20.f);
		points.emplace_back(-30.f , 20.f);
		points.emplace_back(-30.f , -20.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	case 1:
		//三角形
		points.emplace_back(0.f, 60.f);
		points.emplace_back(-52.f, -30.f);
		points.emplace_back(52.f, -30.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	case 2:
		//床
		points.emplace_back(0.f, 0.f);
		points.emplace_back((float)WIN_SIZE_X, 0.f);
		points.emplace_back(0.f, 30.f);
		points.emplace_back((float)WIN_SIZE_X, 30.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, false);
		break;
	case 3:
		//直角三角形
		points.emplace_back(-40.f, 30.f);
		points.emplace_back(-40.f, -30.f);
		points.emplace_back(40.f, -30.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	case 4:
		//直角三角形
		points.emplace_back(-80.f, 180.f);
		points.emplace_back(-80.f, -180.f);
		points.emplace_back(80.f, -180.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	case 5:
		//正三角形
		points.emplace_back(0.f, 15.f);
		points.emplace_back(17.32f, -15.f);
		points.emplace_back(-17.32f, -15.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	case 7:
		//正三角形
		points.emplace_back(200.f, 520.f);
		points.emplace_back(152.f, 437.f);
		points.emplace_back(248.f, 437.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, true);
		break;
	default:
		//床
		points.emplace_back(0.f, 0.f);
		points.emplace_back((float)WIN_SIZE_X, 0.f);
		points.emplace_back(0.f, 50.f);
		points.emplace_back((float)WIN_SIZE_X, 50.f);
		con = new Convex(points, 0.f, 0.f, 0.f, 0.f, false);
		break;
	}
	return con;
}