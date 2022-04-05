/*
 *  aGraphics.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/7/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_GRAPHICS_H
#define ARTEMIS_GRAPHICS_H

#include <SDL.h>
#include <gl/glew.h>
#include <SDL_opengl.h>
#include <gl/glu.h>
#include <SDL_thread.h>
#include "measure.h"
#include "aCamera.h"
#include "aTypewriter.h"

class aGraphics {
private:
	int width;
	int height;
	int bpp;
	bool windowOk;
	bool isWindowed;
	SDL_Window* window;
	SDL_GLContext context;
	//SDL_Surface * screen;
	SDL_Color bgColor;
	//SDL_sem * screenLock;
protected:
public:
	aTypewriter * hTypewriter;

	aGraphics();
	~aGraphics();
	bool setScreen(int newHeight, int newWidth, int newBpp);
	bool declareSettings();
	bool toggleFullscreen();
	static void printAttributes();
	void resize(int width, int height);
	void swapBuffers();
	void clearScreen();

	// Accessors
	int getWidth() { return width; }
	int getHeight() { return height; }
	int getBpp() { return bpp; }
	bool getWindowed() { return isWindowed; }
	float getAspectRatio() { return float(width) / float(height); }

	// Rendering
	void testLoop();
	void drawCube(float xPos, float yPos, float zPos);
	void drawTexturedCube(float xPos, float yPos, float zPos);

	// Manage mode
	void go2d();
	void go3d();
	void go3d(aCamera * cam);
};

#endif