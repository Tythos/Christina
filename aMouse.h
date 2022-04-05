#pragma once

#include "SDL.h"
#include "aGraphics.h"

enum MouseState { NO_BUTTONS, LEFT_BUTTON, RIGHT_BUTTON, BOTH_BUTTONS };

class aMouse
{
private:
	int currX;
	int currY;
	int prevX;
	int prevY;
	MouseState currState;
	MouseState prevState;
protected:
public:
	aMouse(void);
	~aMouse(void);
	void update();
	void update(SDL_Event eve);

	int getCurrX() { return currX; }
	int getCurrY() { return currY; }
	int getPrevX() { return prevX; }
	int getPrevY() { return prevY; }
	int getDX() { return currX - prevX; }
	int getDY() { return currY - prevY; }
	ScreenDimension getXCoord();
	ScreenDimension getYCoord();
	MouseState getCurrState() { return currState; }
	MouseState getPrevState() { return prevState; }
};
