#include "aPlanet.h"

aPlanet::aPlanet()
: aObject() {
	tex = new aTexture();
	tex->loadFromFile("earth.PNG");
	radius = 1.0f;
	resolution = 32;
	mesh = new aMesh();
	mesh->loadSphere(radius, resolution);
}

aPlanet::aPlanet(float r, int n) {
	tex = new aTexture();
	tex->loadFromFile("earth.PNG");
	radius = r;
	resolution = n;
	mesh = new aMesh();
	mesh->loadSphere(radius, resolution);
}

aPlanet::~aPlanet()
{
}

void aPlanet::setRadius(float r) {
	radius = r;
	mesh->loadSphere(radius, resolution);
}

void aPlanet::setResolution(int n) {
	resolution = n;
	mesh->loadSphere(radius, resolution);
}
