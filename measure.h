#ifndef MEASUREMENTS_UTILITY_H
#define MEASUREMENTS_UTILITY_H

enum Alignment { ALIGN_NEGATIVE, ALIGN_MIDDLE, ALIGN_POSITIVE };
enum ScreenUnit { UNIT_PIX, UNIT_PCT };

struct ScreenDimension {
	float value;
	Alignment align;
	ScreenUnit unit;
	ScreenDimension() {
		value = 0.0f;
		align = ALIGN_NEGATIVE;
		unit = UNIT_PIX;
	}
	float toPct(float size) {
		if (unit == UNIT_PIX) {
			return value / size;
		} else {
			return value;
		}
	}
	float toPix(float size) {
		if (unit == UNIT_PIX) {
			return value;
		} else {
			return value * size;
		}
	}
	float toPct(int size) {
		float s = (float)(size);
		return toPct(s);
	}
	float toPix(int size) {
		float s = (float)(size);
		return toPix(s);
	}
};

#endif