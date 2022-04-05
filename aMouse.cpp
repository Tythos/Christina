#include "aMouse.h"

aMouse::aMouse(void)
{
	currX = 0;
	currY = 0;
	prevX = 0;
	prevY = 0;
	currState = NO_BUTTONS;
}

aMouse::~aMouse(void)
{
}

void aMouse::update() {
	// Set mouse for new loop (move curr to prev)
	prevX = currX;
	prevY = currY;
	prevState = currState;
}

void aMouse::update(SDL_Event eve) {
	// Get current mouse
	switch (eve.type) {
		case SDL_MOUSEBUTTONDOWN:
			if (eve.button.button == SDL_BUTTON_LEFT) {
				if (currState == RIGHT_BUTTON) {
					currState = BOTH_BUTTONS;
				} else {
					currState = LEFT_BUTTON;
				}
			} else if (eve.button.button == SDL_BUTTON_RIGHT) {
				if (currState == LEFT_BUTTON) {
					currState = BOTH_BUTTONS;
				} else {
					currState = RIGHT_BUTTON;
				}
			}
			break;
		case SDL_MOUSEBUTTONUP:
			currState = NO_BUTTONS;
			break;
		case SDL_MOUSEMOTION:
			currX = eve.motion.x;
			currY = eve.motion.y;
			break;
		default:
			break;
	}
}

ScreenDimension aMouse::getXCoord() {
	// Mouse coord in pix?
	ScreenDimension toReturn;
	toReturn.align = ALIGN_MIDDLE;
	toReturn.unit = UNIT_PIX;
	toReturn.value = (float)(this->getCurrX());
	return toReturn;
}

ScreenDimension aMouse::getYCoord() {
	// Mouse coord in pix?
	ScreenDimension toReturn;
	toReturn.align = ALIGN_MIDDLE;
	toReturn.unit = UNIT_PIX;
	toReturn.value = (float)(this->getCurrY());
	return toReturn;
}
