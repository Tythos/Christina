#include "aSkybox.h"

aSkybox::aSkybox()
: aObject() {
	tex = new aTexture();
	tex->loadFromFile("earth.PNG");
	radius = 1.0f;
	resolution = 32;
	mesh = new aMesh();
	mesh->loadSphere(radius, resolution);
	ambientMaterial->set(1.0f, 1.0f, 1.0f, 1.0f);
	diffuseMaterial->set(0.0f, 0.0f, 0.0f, 1.0f);
}

aSkybox::aSkybox(float r, int n) {
	tex = new aTexture();
	tex->loadFromFile("earth.PNG");
	radius = r;
	resolution = n;
	mesh = new aMesh();
	mesh->loadSphere(radius, resolution);
	ambientMaterial->set(1.0f, 1.0f, 1.0f, 1.0f);
	diffuseMaterial->set(0.0f, 0.0f, 0.0f, 1.0f);
}

void aSkybox::setRadius(float r) {
	radius = r;
	mesh->loadSphere(radius, resolution);
}

void aSkybox::setResolution(int n) {
	resolution = n;
	mesh->loadSphere(radius, resolution);
}