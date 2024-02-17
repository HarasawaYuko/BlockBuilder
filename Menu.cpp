#include "Menu.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "Share.h"


//メニュー画面

//UIデザイン用定数
static const int ButtonWidth = 250;
static const int StartButtonY = 240;
static const int SetButtonY = 280;
static const int FinButtonWidth = 50;
static const int FinButtonY = 590;
static const int FinButtonX = 740;

Menu::Menu(SceneChanger* changer)
	:BaseScene(changer)
{}

Menu::~Menu()
{}

void Menu::Initialize() {
	//画像・音声のロード
	backPic = LoadGraph("pic/Menu/Back.png");
	bgm = LoadSoundMem("snd/Menu/bgm.mp3");
	int startPic = LoadGraph("pic/Menu/StartButton.png");
	int startPicOn = LoadGraph("pic/Menu/StartButtonOn.png");
	int startSnd = LoadSoundMem("snd/Menu/StartButton.mp3");
	int setPic = LoadGraph("pic/Menu/SettingButton.png");
	int setPicOn = LoadGraph("pic/Menu/SettingButtonOn.png");
	int setSnd = LoadSoundMem("snd/Menu/SettingButton.mp3");
	int finPic = LoadGraph("pic/Menu/FinButton.png");
	int finPicOn = LoadGraph("pic/Menu/FinButtonOn.png");

	//BGMを再生
	ChangeVolumeSoundMem(160 , bgm);
	PlaySoundMem(bgm , DX_PLAYTYPE_BACK, true);

	//ボタンの作成
	startButton = Button(startPic , startPicOn ,startSnd , StartButtonY , ButtonWidth );
	//setButton = Button(setPic , setPicOn , setSnd ,SetButtonY , ButtonWidth);
	finButton = Button(finPic , finPicOn , -1 , FinButtonX , FinButtonY , FinButtonWidth , FinButtonWidth);
}

void Menu::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	//ボタンのupdate
	startButton.update();
	setButton.update();
	finButton.update();

	if (startButton.isPush()) {
		m_sceneChanger->ChangeScene(Scene_Game);
	}
	/*if (setButton.isPush()) {
		m_sceneChanger->ChangeScene(Scene_Setting);
	}*/
	if (finButton.isPush()) {
		m_sceneChanger->ChangeScene(Scene_Fin);
	}
}

void Menu::Draw() {
	//描画モード
	SetDrawMode(DX_DRAWMODE_BILINEAR);
	//背景の描画
	DrawExtendGraph(0, 0, WIN_SIZE_X, WIN_SIZE_Y, backPic, true);
	
	//ボタンの描画
	startButton.draw();
	//setButton.draw();
	finButton.draw();
}

void Menu::Finalize() {
	//ロード画面の表示
	DrawExtendGraph(0, 0, WIN_SIZE_X, WIN_SIZE_Y, Share::loadPic, true);

	//BGMの停止
	StopSoundMem(bgm);
	//画像削除
	deleteMem();
	//ボタンのFinalize
	startButton.finalize();
	//setButton.finalize();
}

//画像、音声のメモリ解放
void Menu::deleteMem() {
	DeleteSoundMem(bgm);
	DeleteGraph(backPic);
}