#ifndef ARTEMIS_GRAPH_H
#define ARTEMIS_GRAPH_H

#include "aColor.h"
#include "aPanel.h"
#include "kString.h"

struct dataPoint {
	float x;
	float y;
	dataPoint * next;

	dataPoint() {
		x = 0.0f;
		y = 0.0f;
		next = NULL;
	}

	dataPoint(float nx, float ny) {
		x = nx;
		y = ny;
		next = NULL;
	}

	dataPoint(const dataPoint &operand) {
		// Deep copy
		x = operand.x;
		y = operand.y;
		if (operand.next != NULL) {
			next = new dataPoint(*(operand.next));
		} else {
			next = NULL;
		}
	}

	dataPoint(int n) {
		// Create numerical function with n points (n,0)
		if (n < 1) {
			return;
		} else if (n == 1) {
			x = (float)n;
			y = 0.0f;
			next = NULL;
			return;
		} else {
			x = (float)n;
			y = 0.0f;
			next = new dataPoint(n-1);
			return;
		}
	}

	~dataPoint() {
		if (next != NULL) {
			delete next;
			next = NULL;
		}
	}

	dataPoint get(int n) {
		dataPoint toReturn;
		if (n == 0) {
			toReturn.x = x;
			toReturn.y = y;
		} else if (next == NULL) {
			toReturn.x = 0.0f;
			toReturn.y = 0.0f;
		} else {
			return next->get(n-1);
		}
		toReturn.next = NULL;
		return toReturn;
	}

	float get(float nx) {
		// Linear interpolation
		if (nx < x || next == NULL) {
			return y;
		} else if (x < nx && nx < next->x) {
			return y + (next->y - y) * (nx - x) / (next->x - x);
		} else {
			return next->get(nx);
		}
	}

	void set(float nx, float ny) {
		// Creates new point (nx,ny) (if nx is not present)
		if (x == nx) {
			// This value of x is already here; replace
			y = ny;
			return;
		} else if (x < nx) {
			if (next == NULL) {
				// Reached end; add new point
				next = new dataPoint(nx, ny);
			} else {
				// More points; continue
				next->set(nx, ny);
			}
		} else {
			// nx < x; insert
			dataPoint * insert = new dataPoint(x, y);
			x = nx; y = ny;
			insert->next = next;
			next = insert;
		}
	}

	void set(int n, float ny) {
		// Sets y value at point n to ny
		if (n == 0) {
			// Set value here
			y = ny;
		} else if (next == NULL) {
			// Index too big; insert here at end
			next = new dataPoint(x + 1.0f, ny);
		} else {
			// Not finished yet; continue
			next->set(n-1, ny);
		}
	}

	int length() {
		if (next == NULL) {
			return 1;
		} else {
			return next->length() + 1;
		}
	}
};

class aGraph : public aPanel {
private:
	aColor * fgColor;
	float margin;
	kString * title;
protected:
public:
	dataPoint * data;
	aGraph(kString t);
	~aGraph();
	void addValue(float x, float y);
	void render(aGraphics * context);
	void setFgColor(float r, float g, float b);
	float getMargin() { return margin; }
	void setMargin(float m);
	void setData(dataPoint * d);
};

#endif