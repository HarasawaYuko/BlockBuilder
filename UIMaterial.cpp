#include "UIMaterial.h"
#include "Mouse.h"
#include "Rand.h"

bool isIn(const int x, const int y, const int width, const int height) {
	if (x < Mouse::instance()->getX() && Mouse::instance()->getX() < x + width
		&& (y - height) < Mouse::instance()->getY() && Mouse::instance()->getY() < y){
		return true;
	}
	else {
		return false;
	}
}

/*****ボタンクラス******/

//全てを指定するコンストラクタ
Button::Button(const int pic , const int onPic , const int sound ,const int x ,const int y , const int width, const int height, const int offPic)
	: pic(pic) ,onPic(onPic),sound(sound), x(x), y(y), width(width), height(height), isOn(false), offPic(offPic)
{}

//左上のyと幅を指定する　中央配置
Button::Button(const int pic, const int onPic, const int sound, const int y, const int width, const int offPic)
	: pic(pic), onPic(onPic), sound(sound), y(y), width(width) , isOn(false), offPic(offPic)
{
	//xの値を設定
	x = (WIN_SIZE_X / 2) - (width / 2);
	//画像サイズを取得
	int x_size;
	int y_size;
	GetGraphSize(pic , &x_size ,&y_size);
	//heightの設定
	float rate = (float)width / (float)x_size;
	height = (int)((float)y_size * rate);
}

void Button::update() {
	if (!active) return;
	push = false;
	isOn = false;
	if (isIn(x, y, width, height)) {
		isOn = true;
		if (Mouse::instance()->isClickNow(LEFT_CLICK)) {
			push = true;
		}
	}
}

void Button::draw() {
	if (!active) {
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), offPic, true);
		return;
	}
	if(isOn){
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), onPic ,true);
	}
	else{
		DrawExtendGraph(x, WIN_SIZE_Y - y , x + width, WIN_SIZE_Y - (y - height), pic, true);
	}
	//押されたら音声を再生
	if (push) {
		PlaySoundMem(sound , DX_PLAYTYPE_BACK, true);
	}
}

void Button::act(const bool a) {
	active = a;
}

bool Button::isPush() {
	return push;
}

void Button::finalize() {
	//画像の削除
	if(pic != -1)DeleteGraph(pic);
	if(onPic != -1)DeleteGraph(onPic);
	if(offPic != -1)DeleteGraph(offPic);
	//音声が再生中か確認する
	while (CheckSoundMem(sound) == 1);
	if(sound != -1)DeleteSoundMem(sound);
}

/**********ラジオボタン***********/

RadioButton::RadioButton(const int x , const int y ,const int size , const int space, const int num ,const int font ,const std::vector<std::string> strs)
	:x(x) , y(y) ,size(size),space(space), num(num), on(-1), fontsize(font), strs(strs)
{}

void RadioButton::draw() const{
	SetFontSize(fontsize);
	//num繰り返し
	for (int i = 0; i < num; i++) {
		DrawCircle(x + size + i*space , y + size,size, COLOR_WHITE, true);
		DrawString(x + size*2 + i*space + 10, y , strs[i].c_str(),COLOR_BLACK );
	}
	if (on >= 0) {
		DrawCircle(x + size + on * space, y + size , size, COLOR_LBLUE , false , 5);
	}
	//選択箇所表示
	DrawCircle(x + size + select * space, y + size, (int)(size * 0.6), COLOR_BLUE, true);
}

int RadioButton::getSelect() const {
	return select;
}

/******スライドバー*******/
SlideBar::SlideBar(const int x , const int y , const int length ,const int size ) 
	:x(x) , y(y) , length(length) , size(size) , on(false), selectX(x)
{}

void SlideBar::update(bool*click) {
	*click = false;
	on = false;
	if (isIn(x , y - size ,length , y+size*2)) {
		on = true;
		if (Mouse::instance()->isClick(LEFT_CLICK)) {
			*click = true;
			selectX = (int)Mouse::instance()->getX();
		}
	}
}

void SlideBar::draw() const{
	DrawBox(x, y, x + length, y + 6 , COLOR_GRAY, true);
	//選択箇所表示
	DrawCircle(selectX, y + 3 , size, COLOR_BLUE, true);
	if (on) {
		DrawCircle(selectX, y+3 , size , COLOR_LBLUE, false, 3);
	}
}

float SlideBar::getRate()const {
	return ((float)(selectX - x)/length);
}

//y軸を反転させた円描画関数
int DrawCircleP(const float x, const float y, const float r , const unsigned int Color , const int FillFrag , const float LineThickness) {
	return DrawCircleAA(x , (float)(WIN_SIZE_Y - y) , r , (int)r, Color, FillFrag, LineThickness);
}

//y軸を反転させた長方形描画関数
int DrawBoxP(const Vec2 cen ,const int cen_r_x , const int cen_r_y ,const double angle ,const int graph , const float h , const float w , const int size_x ,const int size_y,const int transFlag , const int turnFlag) {
	//画像の中心座標を取得
	double exRateX = (double)w/(double)size_x;
	double exRateY = (double)h/(double)size_y;
	return DrawRotaGraph3((int)cen.x_ , WIN_SIZE_Y - (int)cen.y_ , cen_r_x , cen_r_y ,exRateX , exRateY , angle , graph , transFlag , turnFlag);
}

//y軸を反転させた三角形の描画関数
int DrawTriP(const Vec2& p0, const Vec2& p1, const Vec2& p2,const unsigned int color , const int fill ,const float thick) {
	return DrawTriangleAA(p0.x_ , WIN_SIZE_Y -  p0.y_ , p1.x_ , WIN_SIZE_Y - p1.y_ ,p2.x_ ,  WIN_SIZE_Y - p2.y_ , color , fill ,thick);
}

//線分の描画
int DrawSegment(const Segment& line, const unsigned int color) {
	return DrawLineAA(line.start_.x_ , WIN_SIZE_Y - line.start_.y_ , line.end_.x_ , WIN_SIZE_Y - line.end_.y_ , color , 3.f);
}

//線分の描画　二点を受け取るオーバーロード
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int color) {
	return DrawLineAA(s.x_ , WIN_SIZE_Y - s.y_ , e.x_ , WIN_SIZE_Y - e.y_ , color ,3.f);
}

//線分の描画　二点を受け取るオーバーロード
int DrawSegment(const Vec2& s, const Vec2& e, const unsigned int color , const float thick) {
	return DrawLineAA(s.x_, WIN_SIZE_Y - s.y_, e.x_, WIN_SIZE_Y - e.y_, color, thick);
}

int DrawPoint(const Vec2& v, const unsigned int color) {
	return DrawCircle((int)v.x_ , WIN_SIZE_Y - (int)v.y_ , 5 , color);
}

int DrawStrP(const Vec2& p , const std::string str , const unsigned int color) {
	return DrawString((int)p.x_ ,WIN_SIZE_Y - (int)p.y_ , str.c_str(), color);
}

std::string FtoStr(const float val) {
	char tmp[255];
	sprintf_s(tmp, "%.0f", val);
	return std::string(tmp);
}

unsigned int getColorRand() {
	Rand* rand = Rand::instance();
	int a = rand->getI(0, 7);
	return colors[a];
}