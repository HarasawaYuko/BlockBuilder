#include "Share.h"

int Share::score;
int Share::loadPic;

void Share::initialize() {
	loadPic = LoadGraph("pic/Load/back.png");
}