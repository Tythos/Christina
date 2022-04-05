/*
 *  aGame.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/7/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_APP_H
#define ARTEMIS_APP_H

#include <fstream>
#include "kString.h"
#include "aGraphics.h"
#include "aObject.h"
#include "aStopwatch.h"
#include "aKeyboard.h"
#include "aEvent.h"
#include "aPanel.h"
#include "aCamera.h"
#include "aLight.h"
#include "aMouse.h"
#include "aConsole.h"
#include "aCursor.h"
#include "aSoundboard.h"

using namespace std;

class aApp {
private:
protected:
	aPanel * panels;
	aObject * origin;
	aLight * lights;
	kQuat * globalAmbient;
	ofstream debugFile;
public:
	// Data
	aGraphics * hGraphics;
	aStopwatch * hStopwatch;
	aKeyboard * hKeyboard;
	aMouse * hMouse;
	aAppState currState;
	eventList * gameEvents;
	aCamera * camera;
	aConsole * hConsole;
	aCursor * hCursor;
	aSoundboard * hSoundboard;
	bool isDebugToFile;
	bool isDebugToConsole;
	bool isLooping;
	bool (*externalRender)(void);
	
	// Structors
	aApp();
	~aApp();
	
	// Functions
	void initialize(int height, int width);
	void execute();
	void terminate();
	void mainLoop();
	void update();
	void render();
	void debug(kString msg);
	void fail(kString msg);

	// Origin management
	void setOriginVisibility(bool isVisible);

	// List management
	void addPanel(aPanel * p);
	void addObject(aObject * o);
	void addLight(aLight * l);
	
	// Global settings
	void setGlobalAmbient(float r, float g, float b);
};

#endif