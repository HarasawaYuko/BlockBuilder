#pragma once
#include "Object.h"
#include "ContactPoint.h"

//Õ“ËƒNƒ‰ƒX

class Collision {
private:
	int contact_num_;
	float friction_;//–€CŒW”
	float e_ = 0.8f;
	std::vector<ContactPoint> contactpoints_;
	Object* obj1_;
	Object* obj2_;

public:
	Collision();
	Collision(Object* , Object*);

	void addCp(ContactPoint);
	void deleteCp(const int);

	Object* getObj1()const;
	Object* getObj2()const;
	float getE()const;
	float getFri()const;
	int getContactNum()const;
	ContactPoint& getCp(const int);

	void setFri(const float );
	void setE(const float);
};