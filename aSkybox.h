#pragma once

#include "aObject.h"

class aSkybox : public aObject
{
private:
	float radius;
	int resolution;
protected:
public:
	aSkybox();
	aSkybox(float r, int n);
	void setRadius(float r);
	void setResolution(int n);
};
