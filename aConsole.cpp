/*
 *  aConsole.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 1/30/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aConsole.h"
#include <string>
#include "aTypewriter.h"

aConsole::aConsole() {
	lines = new cLine("Console initialized");
	state = CS_DOWN;
	speed.unit = UNIT_PCT; speed.value = 20.0f;	// Default speed is 20% of screen height per second
	limit.unit = UNIT_PCT; limit.value = 0.5f;	// Limit console to covering top half of screen
	x.unit = UNIT_PCT; x.value = 0.0f; x.align = ALIGN_NEGATIVE;
	y.unit = UNIT_PCT; y.value = limit.value; y.align = ALIGN_NEGATIVE;
	w.unit = UNIT_PCT; w.value = 1.0f; w.align = ALIGN_NEGATIVE;
	h.unit = UNIT_PCT; h.value = limit.value; h.align = ALIGN_NEGATIVE;
}

aConsole::~aConsole() {
	if (lines != NULL) {
		delete lines;
		lines = NULL;
	}
}

void aConsole::addLine(kString contents) {
	// Add new line to beginning of stack
	if (lines == NULL) {
		lines = new cLine(contents);
	} else {
		cLine * oldLines = lines;
		lines = new cLine(contents);
		lines->next = oldLines;
	}
}

kString aConsole::getLine(int n) {
	// Returns the nth line, or "" if index is invalid
	if (lines == NULL) { return ""; }
	else { return lines->get(n)->contents; }
}

void aConsole::render(aGraphics * context) {
	// Draw background using panel render
	switch(state) {
		case CS_UP:
			// Don't draw anything
			break;
		case CS_DOWN:
		case CS_MOVING_UP:
		case CS_MOVING_DOWN:
		default:
			// Update font, color, size
			if (font != "") {
				context->hTypewriter->setFont(font);
				font = "";
			}
			context->hTypewriter->setColor(fontColor.r, fontColor.g, fontColor.b);
			context->hTypewriter->setFontSize(fontSize);

			// Draw console background (panel)
			aPanel::render(context);

			// Draw text from bottom up
			int n = lines->numLines();
			ScreenDimension cx = x;
			ScreenDimension cy = y;
			kString line;
			for (int i = 0; i < n && y.value < 1.0f && cy.value < 1.0f; i++) {
				context->hTypewriter->moveCursor(cx, cy);
				line = lines->get(i)->contents;
				context->hTypewriter->type(line, context->getWidth(), context->getHeight());
				cy.value += context->hTypewriter->getFontSize();
			}
			break;
	}
}

void aConsole::update(float dt) {
	// Update panel position based on state and time passed
	switch(state) {
		case CS_MOVING_DOWN:
			// Move panel down by increment
			y.value -= speed.value * dt;
			if (y.value <= limit.value) {
				state = CS_DOWN;
				y.value = limit.value;
			}
			break;
		case CS_MOVING_UP:
			// Move panel up by increment
			y.value += speed.value * dt;
			if (y.value >= 1.0f) {
				state = CS_UP;
				y.value = 1.0f;
			}
			break;
		case CS_UP:
		case CS_DOWN:
		default:
			// Panel remains visible / hidden
			break;
	}
}

void aConsole::setFont(kString f) {
	font = f;
}

void aConsole::setFontColor(float r, float g, float b) {
	fontColor.r = r;
	fontColor.g = g;
	fontColor.b = b;
}

void aConsole::setFontSize(float pct) {
	fontSize = pct;
}