/*
 *  aKeyboard.h
 *  sandbox
 *
 *  Created by Brian Kirkpatrick on 10/22/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_KEYBOARD_H
#define ARTEMIS_KEYBOARD_H

#include "SDL.h"

enum eKeyState {AKEY_NONE, AKEY_NUM, AKEY_CAPS, AKEY_CTRL, AKEY_SHIFT, AKEY_ALT, AKEY_COMMAND, AKEY_WINDOWS};

class aKeyboard {
private:
	bool map[312];
	bool prevMap[312];
	SDL_Event event;
protected:
public:
	aKeyboard();
	void update();
	void update(SDL_Event eve);
	bool checkPressDown(char key);
	bool checkLiftUp(char key);
	bool checkPrevKey(char key);
	bool checkKey(char key);
	bool checkKey(SDL_Keycode aKey);
	eKeyState checkState();
};

#endif