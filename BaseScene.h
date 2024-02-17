#pragma once

#include "Task.h"
#include "SceneChanger.h"

//シーンの基底クラス

class BaseScene : public Task {
protected:
	//シーン変更用
	SceneChanger* m_sceneChanger;

public:
	BaseScene(SceneChanger*);
	virtual ~BaseScene();
	virtual void Initialize() override;
	virtual void Finalize() override;
	virtual void Update() override;
	virtual void Draw() override;
};