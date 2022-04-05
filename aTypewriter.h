/*
 *  Typewriter.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 12/3/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_TYPEWRITER_H
#define ARTEMIS_TYPEWRITER_H

#include "SDL.h"
#include "SDL_ttf.h"
#include "measure.h"
#include "kVec.h"
#include "aColor.h"
#include "kString.h"

using namespace std;

class aTypewriter {
private:
	TTF_Font * currFont;
	kString currFontName;
	aColor * color;
	ScreenDimension cursorX;
	ScreenDimension cursorY;
	ScreenDimension fontSize;
	int nextpoweroftwo(int x);
	
protected:
public:
	aTypewriter();
	~aTypewriter();
	bool setFont(kString fontName);
	bool setColor(float r, float g, float b);
	void setFontSize(float fs);
	float getFontSize() { return fontSize.value; }
	bool stamp(kString text);
	bool type(kString text, int contextWidth, int contextHeight);
	void moveCursor(ScreenDimension x, ScreenDimension y); // Moves cursor to given screen coordinates
};

#endif