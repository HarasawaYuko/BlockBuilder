#pragma once
#include <random>

//—”ƒNƒ‰ƒX

class Rand {
private:
	std::random_device rd;
	std::mt19937 mt;
public:
	static Rand* instance();
	float getF(const int , const int);
	int getI(const int, const int);
};