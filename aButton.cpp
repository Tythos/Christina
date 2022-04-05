#include "aButton.h"

aButton::aButton(void) {
	images[AB_DISABLED] = NULL;
	images[AB_NORMAL] = NULL;
	images[AB_OVER] = NULL;
	images[AB_DOWN] = NULL;
	prevState = AB_NORMAL;
	nextState = AB_NORMAL;
	isSingular = true;
	target = NULL;
}

aButton::~aButton(void) {
	// Release images? Target?
}

bool aButton::isEnabled() {
	return prevState != AB_DISABLED;
}

bool aButton::getIsSingular() {
	return isSingular;
}

void aButton::setState(ButtonState state) {
	nextState = state;
}

void aButton::render(aGraphics * context) {
	// Update state and copy relevant texture to tex, if changed
	if (prevState != nextState && images[nextState] != NULL) {
		tex->loadFromSurface(images[nextState]);
		prevState = nextState;
	}
	aPanel::render(context);
}

void aButton::update(float dt) {
	// Don't check mouse events / target here; use external game events instead
}

void aButton::setImage(ButtonState state, SDL_Surface * image) {
	images[state] = image;
	if (state == nextState) {
		tex->loadFromSurface(image);
	}
}