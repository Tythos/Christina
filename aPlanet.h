#pragma once

#include "aObject.h"

class aPlanet : public aObject
{
private:
	float radius;
	int resolution;
protected:
public:
	aPlanet();
	aPlanet(float r, int n);
	~aPlanet();
	void setRadius(float r);
	void setResolution(int n);
};
