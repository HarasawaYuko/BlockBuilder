#include "DxLib.h"
#include "SceneMgr.h"
#include "FpsControl.h"
#include "Share.h"

//定数
//ウィンドウの初期位置
#define WIN_POS_X 0
#define WIN_POS_Y 0
#define WIN_SIZE_X 800
#define WIN_SIZE_Y 600

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    // ウィンドウモードに設定
    ChangeWindowMode(TRUE);
    SetWaitVSyncFlag(FALSE);
    // DXライブラリ初期化処理
    if (DxLib_Init() < 0) {
        //エラーなら終了する
        return -1;
    }
    //描画先を裏画面にする
    SetDrawScreen(DX_SCREEN_BACK);
    //画面等の設定
    SetWindowText("Physics Engine");
    SetGraphMode(WIN_SIZE_X , WIN_SIZE_Y , 32);
    SetWindowInitPosition(WIN_POS_X , WIN_POS_Y);
    SetBackgroundColor(255 , 255 , 255);
    ChangeFontType(DX_FONTTYPE_ANTIALIASING);
    ChangeFont("Meiryo UI");

    //xボタンで終了しない
    SetWindowUserCloseEnableFlag(false);

    SceneMgr sceneMgr;
    FpsControl fpsControl;

    //初期化
    sceneMgr.Initialize();
    Share::initialize();

    // while(裏画面を表画面に反映, メッセージ処理, 画面クリア)
    while ( ProcessMessage() == 0 && ClearDrawScreen() == 0) {
        fpsControl.Update();
        sceneMgr.Update();
        sceneMgr.Draw();
        ScreenFlip();
        fpsControl.Wait();
        if (sceneMgr.fin) break;
    }
    sceneMgr.Finalize();
    DxLib_End();    // DXライブラリ終了処理
    return 0;
}