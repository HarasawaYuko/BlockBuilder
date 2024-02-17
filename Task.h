#pragma once

//’ŠÛƒNƒ‰ƒX

class Task {
public:
	//À‘•‚µ‚È‚­‚Ä‚à—Ç‚¢
	virtual void Initialize(){}
	virtual void Finalize(){}
	//À‘•‚ª•K{
	virtual void Update() = 0;
	virtual void Draw() = 0;
};