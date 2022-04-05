/*
 *  aMesh.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 7/27/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_MESH_H
#define ARTEMIS_MESH_H

class aVertex {
public:
	float x;
	float y;
	float z;
	
	aVertex();
	aVertex(float px, float py, float pz);
	void set(float px, float py, float pz);
	float mag();
	aVertex operator- (aVertex operand);
	bool operator== (aVertex operand);
	aVertex cross(aVertex operand);
	aVertex norm();
	void print();
};

class aTexCoord {
public:
	float u;
	float v;
};

class aMesh {
private:
	int _numVertices;
	aVertex * _vertices;
	aVertex * _normals;
	aTexCoord * _texCoords;
protected:
public:
	// Constructors
	aMesh();
	~aMesh();
	
	// Loaders
	bool loadCube(float size); // Loads vertices defining size x size x size cube, where size is the length of one side.
	bool loadTetra(float size); // Loads vertices defining size x size x size tetrahedral, where size is the length of one side
	bool loadSphere(float size, int numFaces); // Loads vertices defining sphere with radius size and the given number of faces
	
	// Methods
	void clear();
	void render();
	void setColor(float r, float g, float b, float a);
	void refreshNormals();
	void debug();
	void disableTexture();
};

#endif