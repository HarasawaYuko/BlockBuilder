#pragma once
#include <functional>

#include "BaseScene.h"
#include "SceneChanger.h"
#include "UIMaterial.h"

//ƒQ[ƒ€‰æ–Ê

class Menu :public BaseScene {
private:
	int backPic;//”wŒi
	int bgm;//BGM
	Button startButton;
	Button setButton;
	Button finButton;
public:
	//ŠÖ”
	void deleteMem();

	Menu(SceneChanger*);
	~Menu();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
};