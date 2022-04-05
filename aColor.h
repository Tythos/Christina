#ifndef ARTEMIS_COLOR_H
#define ARTEMIS_COLOR_H

class aColor {
public:
	float r;
	float g;
	float b;
	float a;
	
	aColor();
	aColor(float fr, float fg, float fb, float fa);
	void set(float fr, float fg, float fb, float fa);
	void setAll();
	void setColor();
	void setDiffuseMat();
	void setAmbientMat();
};

#endif
