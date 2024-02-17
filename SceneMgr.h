#pragma once

#include "BaseScene.h"
#include "SceneMgr.h"

class SceneMgr :public SceneChanger, Task {
private:
	BaseScene* m_scene;//現在のシーンインスタンス
	Scene m_next_scene; //次のシーン

public:
	SceneMgr();
	void Initialize() override;
	void Finalize() override;
	void Update() override;
	void Draw() override;
	bool fin = false;
	
	//シーンの変更
	void ChangeScene(Scene next) override;
};