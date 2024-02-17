#pragma once
#include "include.h"
#include "UIMaterial.h"

class FpsControl {
private:
	int startTime;//測定開始時刻
	int count;//カウンタ
	float fps;//実際のfps
	static const int N = 60;//サンプル数
public:
	FpsControl();
	void Update();
	void Draw();
	void Wait();
};