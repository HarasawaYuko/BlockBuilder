#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "UIMaterial.h"

//ƒQ[ƒ€‰æ–Ê

class DetectTest :public BaseScene {
private:
public:
	//ŠÖ”
	void deleteMem();

	DetectTest(SceneChanger*);
	~DetectTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};