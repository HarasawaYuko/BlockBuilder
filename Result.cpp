#include "Result.h"
#include "KeyBoard.h"
#include "Mouse.h"
#include "Rand.h"
#include "Share.h"

static const int NextButtonY = 210;
static const int RetryButtonY = 290;
static const int ButtonWidth = 250;
static const int ScoreY = 420;

Result::Result(SceneChanger* changer)
	:BaseScene(changer)
{}

Result::~Result()
{}

void Result::Initialize() {
	//画像と音声のロード
	backPic = LoadGraph("pic/Result/Back.png");
	bgm = LoadSoundMem("snd/Result/bgm.mp3");
	int nextPic = LoadGraph("pic/Result/next.png");
	int nextOnPic = LoadGraph("pic/Result/nextOn.png");
	int nextSnd = LoadSoundMem("snd/Result/next.mp3");
	int retryPic = LoadGraph("pic/Result/retry.png");
	int retryOnPic = LoadGraph("pic/Result/retryOn.png");
	int retrySnd = LoadSoundMem("snd/Result/retry.mp3");



	//BGMを再生
	ChangeVolumeSoundMem(160, bgm);
	PlaySoundMem(bgm, DX_PLAYTYPE_BACK, true);
	
	//ボタンの作成
	nextButton = Button(nextPic , nextOnPic ,nextSnd ,NextButtonY ,ButtonWidth);
	retryButton = Button(retryPic , retryOnPic , retrySnd , RetryButtonY , ButtonWidth);
}

//円の並進運動
void Result::Update() {
	//入力状態の取得
	KeyBoard::instance()->update();
	Mouse::instance()->update();

	//ボタンの更新
	nextButton.update();
	retryButton.update();

	if (nextButton.isPush()) {
		m_sceneChanger->ChangeScene(Scene_Menu);
	}
	if (retryButton.isPush()) {
		m_sceneChanger->ChangeScene(Scene_Game);
	}
}

void Result::Draw() {
	//背景の描画
	DrawExtendGraph(0, 0, WIN_SIZE_X, WIN_SIZE_Y, backPic, true);

	SetFontSize(20);
	//ボタンの描画
	nextButton.draw();
	retryButton.draw();
	
	SetFontSize(90);
	std::string scoreStr = std::to_string(Share::score);
	int length = GetDrawStringWidth(scoreStr.c_str(), (int)scoreStr.length());
	DrawFormatString((WIN_SIZE_X - length)/2 , WIN_SIZE_Y - ScoreY , COLOR_BLACK , "%d" , Share::score);
	SetFontSize(40);
	DrawString(WIN_SIZE_X/2 + length/2 , WIN_SIZE_Y- ( ScoreY - (90-40)), "pt", COLOR_BLACK);
}

void Result::Finalize() {
	//ロード画面の表示
	DrawExtendGraph(0, 0, WIN_SIZE_X, WIN_SIZE_Y, Share::loadPic, true);
	//BGMの停止
	StopSoundMem(bgm);
	//メモリ削除
	nextButton.finalize();
	retryButton.finalize();
}

//画像、音声のメモリ解放
void Result::deleteMem() {
	DeleteSoundMem(bgm);
	DeleteGraph(backPic);
}