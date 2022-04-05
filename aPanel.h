/*
 *  aPanel.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/8/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_PANEL_H
#define ARTEMIS_PANEL_H

#include "kString.h"
#include "kVec.h"
#include "SDL.h"
#include "aGraphics.h"
#include "aTexture.h"
#include "aColor.h"

using namespace std;

class aPanel {
private:
protected:
	aPanel * next;		// Subsequent siblings
	aPanel * children;	// Panels rendered relative to this one
	aTexture * tex;
	aColor * bgColor;
	ScreenDimension x;
	ScreenDimension y;
	ScreenDimension w;
	ScreenDimension h;
	bool visible;
public:
	bool isTranslucent;

	// Constructors
	aPanel();
	aPanel(float nx, float ny, float nw, float nh);
	~aPanel();

	// Virtual functions
	virtual void update(float dt);
	virtual void render(aGraphics * context);
	virtual void setImage(kString filename);
	virtual void setImage(SDL_Surface * surf);
	
	// Listing
	aPanel * get(int n);
	void set(aPanel * value);
	int listLength();

	// Accessors
	ScreenDimension getX() { return x; }
	ScreenDimension getY() { return y; }
	ScreenDimension getW() { return w; }
	ScreenDimension getH() { return h; }
	bool getVisible() { return visible; }
	void setX(float v);
	void setY(float v);
	void setW(float v);
	void setH(float v);
	void setVisible(bool v);
	void setBgColor(float r, float g, float b, float a);

	// Collision Detection
	bool isWithin(ScreenDimension xCoord, ScreenDimension yCoord, aGraphics * context);
};

#endif