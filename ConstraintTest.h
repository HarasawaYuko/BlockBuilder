#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "World.h"

//ƒQ[ƒ€‰æ–Ê

class ConstraintTest :public BaseScene {
private:
	World world;
public:
	//ŠÖ”
	void deleteMem();

	ConstraintTest(SceneChanger*);
	~ConstraintTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};