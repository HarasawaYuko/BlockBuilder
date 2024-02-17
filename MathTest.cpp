#include "MathTest.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "Convex.h"

//数値計算等テスト用
static int mode = 0;

//点と直線の距離 mode = 0
static Segment line;
static Vec2 point;
static Vec2 contactPoint;
static bool start = true;
static float dis;
static int  pattern;

//なす角 mode = 1
static bool p0set = true;
static Vec2 center =Vec2(400 ,300);
static Vec2 p0;
static Vec2 p1;
static float theta;

//Convexコンストラクタ mode = 2
static std::vector<Vec2> points;
static Convex* convex;
static bool make =  false;

//Matrix mode = 3
static Matrix* matrix;
static Matrix* mat1 = new Matrix(2 ,2);
static Matrix* mat2 = new Matrix(2, 2);
static Matrix pro = Matrix(0 ,0);
static bool makeM = false;
static int row = 2;//列
static int column = 3;//行

MathTest::MathTest(SceneChanger* changer)
	:BaseScene(changer)
{}

MathTest::~MathTest()
{}

void MathTest::Initialize() {
}

void MathTest::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	if (KeyBoard::instance()->isHitNow(KEY_INPUT_S)) {
		line.start_ = Vec2();
		line.end_ = Vec2();
	}
	
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_P)) {
		mode++;
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_M)) {
		mode--;
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_LEFT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Detect);
	}
	if (KeyBoard::instance()->isHitNow(KEY_INPUT_RIGHT)) {
		m_sceneChanger->ChangeScene(Scene_TEST_Constraint);
	}

	switch (mode) {
	case 0://点と線分の距離
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			if (start) {
				line.start_.set(Mouse::instance()->getX(), Mouse::instance()->getY());
			}
			else {
				line.end_.set(Mouse::instance()->getX(), Mouse::instance()->getY());
			}
			start = !start;
		}
		else if (Mouse::instance()->isClickNow(RIGHT_CLICK)) {
			point.set(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		{
			if (KeyBoard::instance()->isHitNow(KEY_INPUT_D)) {
				dis = distanceSegPoint(point, line);
				Vec2 edgeVec = (line.end_ - line.start_).normalize();
				Vec2 StoP = point - line.start_;
				//始点からの距離を求める
				float dis_ = StoP.dot(edgeVec);
				contactPoint = line.start_ + (edgeVec * dis_);
			}
		}
		break;
		//なす角
	case 1:
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			
			p0.set(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		else if (Mouse::instance()->isClickNow(RIGHT_CLICK)) {
			p1.set(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		theta = DegToRad(calAngle(center , p0 , p1));
		break;
	case 2:
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			points.emplace_back(Mouse::instance()->getX(), Mouse::instance()->getY());
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_L)) {
			delete convex;
			make = false;
			points.clear();
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN)) {
			convex = new Convex(points);
			make = true;
		}
		break;
	case 3:
		//行列
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_RETURN)) {
			mat1->matrix_[0][0] = 1.f;
			mat1->matrix_[0][1] = 2.f;
			mat1->matrix_[1][0] = 3.f;
			mat1->matrix_[1][1] = 4.f;
			mat2->matrix_[0][0] = 3.f;
			mat2->matrix_[0][1] = 2.f;
			mat2->matrix_[1][0] = 5.f;
			mat2->matrix_[1][1] = 6.f;
			pro = (*mat1)+(*mat2);
		}
		if (KeyBoard::instance()->isHitNow(KEY_INPUT_D)) {
			delete matrix;
		}
	}
}

void MathTest::Draw() {
	SetFontSize(20);
	DrawFormatString(640, 0, COLOR_BLACK, "MathTest %d",mode );
	DrawString(640, 23, "S->Reset" , COLOR_BLACK);

	switch (mode) {
	case 0:
		DrawString(0, 23, "点と直線の距離", COLOR_BLACK);
		DrawSegment(line, COLOR_BLUE);
		DrawPoint(point, COLOR_RED);
		DrawPoint(contactPoint, COLOR_GREEN);
		SetFontSize(30);
		DrawFormatString(0, 300, COLOR_RED, "距離:%f", dis);
		DrawFormatString(0, 330, COLOR_RED, "パターン:%d", pattern);
		SetFontSize(20);
		break;
	case 1:
		DrawString(0, 23, "rightClick->緑 leftClick->青", COLOR_BLACK);
		DrawString(0, 50, "なす角 青から緑まで反時計周り\n", COLOR_BLACK);
		DrawPoint(center, COLOR_RED);
		DrawSegment(Segment(center, p0), COLOR_BLUE);
		DrawSegment(Segment(center, p1), COLOR_GREEN);
		SetFontSize(40);
		DrawFormatString(0, 70, COLOR_BLACK, "Theta = %f", theta);
		break;
	case 2:
		DrawString(0, 23, "leftClick->点の追加 L->クリア Enter->Convex作成", COLOR_BLACK);
		DrawString(0, 50, "Convexコンストラクタ\n", COLOR_BLACK);
		for (int i = 0; i < points.size(); i++) {
			DrawPoint(points[i], COLOR_RED);
		}
		if (make) {
			convex->Draw();
			DrawPoint(convex->getCenter(), COLOR_ORANGE);
			for (int i = 0; i < convex->getPointNum(); i++) {
				Vec2 p = convex->getPointW(i);
				DrawFormatString((int)p.x_, WIN_SIZE_Y - (int)p.y_, COLOR_BLUE, "%d", i);
			}
		}
		break;
	case 3:
		DrawString(0 ,0 , "行列演算" , COLOR_BLACK);
		if (true) {
			DrawFormatString(100, 200, COLOR_BLUE, "%s", mat1->toString().c_str());
			DrawFormatString(300, 200, COLOR_BLUE, "%s", mat2->toString().c_str());
			DrawFormatString(500, 200, COLOR_BLACK, "%s", pro.toString().c_str());
		}
	}


}

void MathTest::Finalize() {
	for (Object* obj : world.objects_) {
		delete obj;
	}
}

//画像、音声のメモリ解放
void MathTest::deleteMem() {
}