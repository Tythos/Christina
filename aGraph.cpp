#include "aGraph.h"

aGraph::aGraph(kString t) {
	title = new kString(t);
	fgColor = new aColor();
	bgColor = new aColor();
	margin = 0.2f;
	data = NULL;
}

aGraph::~aGraph(void) {
	if (fgColor != NULL) {
		delete fgColor;
		fgColor = NULL;
	}
	if (bgColor != NULL) {
		delete bgColor;
		bgColor = NULL;
	}
	if (data != NULL) {
		delete data;
		data = NULL;
	}
	if (title != NULL) {
		delete title;
		title = NULL;
	}
}

void aGraph::addValue(float x, float y) {
	if (data == NULL) {
		// Create first point
		data = new dataPoint(x, y);
	} else if (data->x > x) {
		// Before first point
		dataPoint * tmp = data;
		data = new dataPoint(x, y);
		data->next = tmp;
	} else if (data->next == NULL) {
		// Only one point
		data->next = new dataPoint(x, y);
	} else {
		dataPoint * tmp1 = data;
		dataPoint * tmp2 = data->next;
		while (tmp1->x < x && tmp2->x < x && tmp2->next != NULL) {
			tmp1 = tmp1->next;
			tmp2 = tmp2->next;
		}
		if (tmp2->next == NULL && tmp2->x < x) {
			// Add at end
			tmp2->next = new dataPoint(x, y);
		} else {
			// Insert between tmp1, tmp2
			dataPoint * insert = new dataPoint(x, y);
			tmp1->next = insert;
			insert->next = tmp2;
		}
	}
}

void aGraph::render(aGraphics * context) {
	if (!visible) { return; }

	// Calculate location, position
	int screenWidth = context->getWidth();
	int screenHeight = context->getHeight();
	float centerX = x.unit == UNIT_PIX ? x.value / ((float)screenWidth) : x.value;
	float centerY = y.unit == UNIT_PIX ? y.value / ((float)screenHeight) : y.value;
	float left, right, bottom, top;
	switch (w.align) {
		case ALIGN_NEGATIVE:
			left = centerX;
			right = w.unit == UNIT_PIX ? centerX + w.value / (float)screenWidth : centerX + w.value;
			break;
		case ALIGN_POSITIVE:
			left = w.unit == UNIT_PIX ? centerX - w.value / (float)screenWidth : centerX - w.value;
			right = centerX;
			break;
		case ALIGN_MIDDLE:
		default:
			left = w.unit == UNIT_PIX ? centerX - 0.5f * w.value / (float)screenWidth : centerX - 0.5f * w.value;
			right = w.unit == UNIT_PIX ? centerX + 0.5f * w.value / (float)screenWidth : centerX + 0.5f * w.value;
			break;
	}
	switch (h.align) {
		case ALIGN_NEGATIVE:
			bottom = centerY;
			top = h.unit == UNIT_PIX ? centerY + h.value / (float)screenHeight : centerY + h.value;
			break;
		case ALIGN_POSITIVE:
			bottom = h.unit == UNIT_PIX ? centerY - h.value / (float)screenHeight : centerY - h.value;
			top = centerY;
			break;
		case ALIGN_MIDDLE:
		default:
			bottom = h.unit == UNIT_PIX ? centerY - 0.5f * h.value / (float)screenHeight : centerY - 0.5f * h.value;
			top = h.unit == UNIT_PIX ? centerY + 0.5f * h.value / (float)screenHeight : centerY + 0.5f * h.value;
			break;
	}

	// Draw 2d quad
	glBindTexture(GL_TEXTURE_2D, 0);
	bgColor->setAll();
	glBegin(GL_QUADS); {
		glVertex2f(left, bottom);
		glVertex2f(right, bottom);
		glVertex2f(right, top);
		glVertex2f(left, top);
	} glEnd();

	// Set up text dimensions
	ScreenDimension cursorX, cursorY;
	cursorX.unit = UNIT_PCT; cursorX.align = ALIGN_NEGATIVE; 
	cursorY.unit = UNIT_PCT; cursorY.align = ALIGN_NEGATIVE; 

	// Draw plot (if there's more than 1 point)
	fgColor->setAll();
	int length = data->length();
	if (length > 1) {
		dataPoint first = data->get(0);
		dataPoint last = data->get(length-1);

		// Determine highest and lowest values for scaling
		float yMax = first.y;
		float yMin = first.y;
		dataPoint tmp;
		for (int i = 0; i < length; i++) {
			tmp = data->get(i);
			if (tmp.y > yMax) { yMax = tmp.y; }
			if (tmp.y < yMin) { yMin = tmp.y; }
		}

		// Plotting area dimensions and location
		float plotW = w.toPct((float)(context->getWidth())) * (1.0f - 2.0f * margin);
		float plotH = h.toPct((float)(context->getHeight())) * (1.0f - 2.0f * margin);
		float plotX = left + w.toPct((float)(context->getWidth())) * margin;
		float plotY = bottom + h.toPct((float)(context->getHeight())) * margin;
		float xSpan = last.x - first.x;
		float ySpan = yMax - yMin;

		// Draw each point
		float screenX, screenY;
		glBegin(GL_LINE_STRIP); {
			for (int i = 0; i < length; i++) {
				tmp = data->get(i);
				screenX = plotX + plotW * tmp.x / xSpan;
				screenY = plotY + plotH * (tmp.y - yMin) / ySpan;
				glVertex2f(screenX, screenY);
			}
		} glEnd();

		// Draw the axes
		glBegin(GL_LINES); {
			glVertex2f(plotX, plotY);
			glVertex2f(plotX + plotW, plotY);
			glVertex2f(plotX, plotY);
			glVertex2f(plotX, plotY + plotH);
		} glEnd();

		// Draw labels
		context->hTypewriter->setColor(fgColor->r, fgColor->g, fgColor->b);
		cursorX.value = plotX; cursorY.value = bottom;
		context->hTypewriter->moveCursor(cursorX, cursorY);
		context->hTypewriter->type(kString(first.x), context->getWidth(), context->getHeight());
		cursorX.value = plotX + plotW - 0.05f; cursorY.value = bottom;
		context->hTypewriter->moveCursor(cursorX, cursorY);
		context->hTypewriter->type(kString(last.x), context->getWidth(), context->getHeight());
		cursorX.value = left; cursorY.value = plotY;
		context->hTypewriter->moveCursor(cursorX, cursorY);
		context->hTypewriter->type(kString(yMin), context->getWidth(), context->getHeight());
		cursorX.value = left; cursorY.value = plotY + plotH - 0.05f;
		context->hTypewriter->moveCursor(cursorX, cursorY);
		context->hTypewriter->type(kString(yMax), context->getWidth(), context->getHeight());
	}

	// Draw the title along the top-left
	cursorX.value = left + 0.01f;
	cursorY.value = top - context->hTypewriter->getFontSize() - 0.01f;
	context->hTypewriter->setColor(fgColor->r, fgColor->g, fgColor->b);
	context->hTypewriter->moveCursor(cursorX, cursorY);
	context->hTypewriter->type(*title, context->getWidth(), context->getHeight());
}

void aGraph::setFgColor(float r, float g, float b) {
	fgColor->set(r, g, b, 1.0f);
}

void aGraph::setMargin(float m) {
	if (0.0f < margin && margin < 0.5f) {
		margin = m;
	}
}

void aGraph::setData(dataPoint * d) {
	// DO NOT delete old data; it was externally created and referenced, assume external creator will clean up
	data = d;
}