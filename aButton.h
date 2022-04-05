/*
 *  aButton.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 2/11/10
 *  Copyright 2010 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *  Defines in-game buttons and links to events
 */

#ifndef ARTEMIS_BUTTON_H
#define ARTEMIS_BUTTON_H

#include "aPanel.h"

enum ButtonState { AB_DISABLED, AB_NORMAL, AB_OVER, AB_DOWN };

class aButton :	public aPanel {
private:
	SDL_Surface * images[4];
	ButtonState prevState;
	ButtonState nextState;
	bool isSingular;
	void (*target)(void);
protected:
public:
	aButton(void);
	~aButton(void);
	bool isEnabled();
	bool getIsSingular();
	void setState(ButtonState state);
	ButtonState getState() { return prevState; }
	void setImage(ButtonState state, SDL_Surface * image);
	void render(aGraphics * context);
	void update(float dt);
};

#endif