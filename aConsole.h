/*
 *  aConsole.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 1/30/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *  First test of aPanel class inheritance
 *  Defines in-game console and link to scripting interpreter
 */

#ifndef ARTEMIS_CONSOLE_H
#define ARTEMIS_CONSOLE_H

#include <stdio.h>
#include "kString.h"
#include "aPanel.h"
#include "aTypewriter.h"
#include "aColor.h"

enum ConsoleState { CS_UP, CS_DOWN, CS_MOVING_UP, CS_MOVING_DOWN };

struct cLine {
	kString contents;
	int ttl;
	cLine * next;
	cLine() {
		contents = "";
		next = NULL;
	}
	cLine(kString stuff) {
		contents = stuff;
		next = NULL;
	}
	~cLine() {
		if (next != NULL) {
			delete next;
			next = NULL;
		}
	}
	int numLines() {
		if (next == NULL) { return 1; }
		else { return next->numLines() + 1; }
	}
	cLine * get(int n) {
		if (n < 0) {
			// Negative index; return this
			return this;
		} else if (next == NULL && n > 0) {
			// Reached end; return last
			return this;
		} else if (n == 0) {
			// Reached desired location; return
			return this;
		} else {
			// Not there yet; increment
			return next->get(n-1);
		}
	}
};

class aConsole : public aPanel {
private:
	cLine * lines;
	ScreenDimension speed; // Speed at which console is animated {unit / sec)
	ScreenDimension limit; // Limit (from top of screen) of visible console
	kString font;
	aColor fontColor;
	float fontSize;
protected:
public:
	ConsoleState state;
	aConsole();
	~aConsole();
	void addLine(kString contents);
	kString getLine(int n);
	void render(aGraphics * context);
	void update(float dt);
	void setFont(kString f);
	void setFontColor(float r, float g, float b);
	void setFontSize(float pct);
};

#endif