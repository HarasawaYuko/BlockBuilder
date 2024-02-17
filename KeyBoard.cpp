#include "KeyBoard.h"

KeyBoard* KeyBoard::instance() {
	static KeyBoard instance;
	return &instance;
}

void KeyBoard::update() {
	//ó‘Ô‚ğæ“¾
	char tmp_key[256];
	GetHitKeyStateAll(tmp_key);
	for (int i = 0 ; i < 256; i++) {
		if (tmp_key[i] == 1) {
			key_state_[i]++;
		}
		else {
			key_state_[i] = 0;
		}
	}
}

bool KeyBoard::isHitNow(const int key) {
	if (key_state_[key] == 1) {
		return true;
	}
	return false;
}

bool KeyBoard::isPush(const int key) {
	if (key_state_[key] > 0) {
		return true;
	}
	return false;
}

int KeyBoard::getFrame(const int key) {
	return key_state_[key];
}