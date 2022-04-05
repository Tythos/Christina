#include "aColor.h"

#include <SDL_opengl.h>

aColor::aColor() { 
	r = 1.0f; g = 1.0f; b = 1.0f; a = 1.0f; 
}

aColor::aColor(float fr, float fg, float fb, float fa) {
	r = fr; g = fg; b = fb; a = fa; 
}

void aColor::set(float fr, float fg, float fb, float fa) { 
	r = fr; g = fg; b = fb; a = fa; 
}

void aColor::setAll() {
	float values[] = { r, g, b, a };
	glColor4f(r, g, b, a);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, values);
	glMaterialfv(GL_FRONT, GL_AMBIENT, values);
}

void aColor::setColor() {
	glColor4f(r, g, b, a);
}

void aColor::setDiffuseMat() {
	float values[] = { r, g, b, a };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, values);
}

void aColor::setAmbientMat() {
	float values[] = { r, g, b, a };
	glMaterialfv(GL_FRONT, GL_AMBIENT, values);
}
