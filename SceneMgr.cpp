#include "SceneMgr.h"
#include "Menu.h"
#include "Game.h"
#include "Result.h"
#include "DetectTest.h"
#include "MathTest.h"
#include "ObjectTest.h"
#include "ConstraintTest.h"
#include "WholeTest.h"


SceneMgr::SceneMgr() {
	m_scene = (BaseScene*)new Menu(this);
}

void SceneMgr::Initialize() {
	m_next_scene = Scene_None;
	m_scene->Initialize();
}

void SceneMgr::Finalize() {
	m_scene->Finalize();
}

void SceneMgr::Update() {
	if (m_next_scene != Scene_None) {
		//現在のシーンの終了処理
		m_scene->Finalize();
		//メモリの解法
		delete m_scene;
		switch (m_next_scene) {
			break;
		case Scene_Menu:
			m_scene = (BaseScene*) new Menu(this);
			break;
		case Scene_Game:
			m_scene = (BaseScene*) new Game(this);
			break;
		case Scene_Result:
			m_scene = (BaseScene*) new Result(this);
			break;
		case Scene_TEST_Object:
			m_scene = (BaseScene*) new ObjectTest(this);
			break;
		case Scene_TEST_Detect:
			m_scene = (BaseScene*) new DetectTest(this);
			break;
		case Scene_TEST_Math:
			m_scene = (BaseScene*) new MathTest(this);
			break;
		case Scene_TEST_Constraint:
			m_scene = (BaseScene*) new ConstraintTest(this);
			break;
		case Scene_TEST_Whole:
			m_scene = (BaseScene*) new WholeTest(this);
			break;
		}
		m_next_scene = Scene_None;
		m_scene->Initialize();
	}
	m_scene->Update();
}

void SceneMgr::Draw() {
	m_scene->Draw();
}

void SceneMgr::ChangeScene(Scene next) {
	if (next == Scene_Fin) fin = true;
	m_next_scene = next;
}