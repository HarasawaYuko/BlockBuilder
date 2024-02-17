#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "World.h"
#include "UIMaterial.h"
#include "Circle.h"

#include "Pair.h"

//ÉQÅ[ÉÄâÊñ 

class WholeTest :public BaseScene {
private:
	World world;
public:
	//ä÷êî
	void deleteMem();

	WholeTest(SceneChanger*);
	~WholeTest();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};