#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "UIMaterial.h"

//ƒQ[ƒ€‰æ–Ê

class ObjectTest :public BaseScene {
private:
public:
	//ŠÖ”
	void deleteMem();

	ObjectTest(SceneChanger*);
	~ObjectTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};
