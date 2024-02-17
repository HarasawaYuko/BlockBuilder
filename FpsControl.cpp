#include "FpsControl.h"

FpsControl::FpsControl()
	:startTime(0) , count(0) , fps(0)
{}

void FpsControl::Update() {
	//計測開始フレームであれば
	if (count == 0) {
		startTime = GetNowCount();
	}
	if (count == N) {
		int time = GetNowCount();//時間を取得
		fps = 1000.f / ((time - startTime) / (float)N);//1秒に何回更新したか
		count = 0;
		//基準時刻を更新
		startTime = time;
	}
	count++;
}

void FpsControl::Draw() {
	SetFontSize(20);
	DrawFormatString(WIN_SIZE_X - 60, WIN_SIZE_Y - 25,COLOR_BLACK , "%.1f", fps);
}

void FpsControl::Wait() {
	//かかった時間
	int tookTime = GetNowCount() - startTime;
	//待つべき時間
	int waitTime = count*1000/FPS - tookTime;
	//待つべき時間があれば
	if (waitTime > 0) {
		Sleep(waitTime);
	}
}