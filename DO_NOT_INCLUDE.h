/*
 *  libArtemis.h
 *  Static library header for external usage
 *
 *  Created by Brian Kirkpatrick on 10/10/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef LIB_ARTEMIS_H
#define LIB_ARTEMIS_H

// Libraries
#include <iostream>
#include <ofstream>
#include <ctime>
#include "SDL.h"
#include <SDL_opengl.h>
#include "SDL_image.h"

// Utility
namespace Conversion {
	char * floatToStr(float myFloat);
	char * intToStr(int myInt);
}

template <class T> class nTree {
private:
	T * pItem;
	nTree * parent;
	smartList<nTree> children;
	int level;
	int nodeId;
protected:
	// Node-level functions, called on a specific tree node
	nTree<T> * getNode(int requestedId);
	nTree<T> * headNode();
	int numChildren();
	int getLevel();
	void printNode();	
public:
	// Constructors
	nTree();
	nTree(T * newItem);
	~nTree();
	
	// Tree-level functions, must always be called on head
	T * getItem(int nodeId);
	bool isNode(int testId);
	int nextAvailableId();
	int getSize();
	int addChild(T * newItem, int parentId);
	int addChild(T * newItem);
	void removeNode(int requestedId);
	void merge(nTree<T> * subTree, int parentId);
	void printNode(int nodeId);
	void printTree();
	void reset();
};

template <class T> class smartList {
private:
	T * pItem;
	smartList * pNext;
	int nToEnd;
	T * get(int n, int length);
	void print(int l);
protected:
public:
	// Constructors
	smartList();
	smartList(const smartList<T> &rhs);
	~smartList();
	
	// Accessors
	T * get(int n);
	int length();
	void add(T * newItem);
	void remove(int n);
	
	// Operators
	void print();
	T& operator[](int offset);
};

// Math
class kVec {
private:
	float * vec;
	int len;
protected:
public:
	// Constructors
	kVec();
	kVec(int length);
	kVec(int length, float vals[]);
	kVec(kVec const& b);
	~kVec();
	
	// Access
	int getLength() { return len; }
	void set(int length, float vals[]);
	
	// Math
	float abs();
	
	// Operators
	friend ostream& operator <<(ostream &os, const kVec &myVec);
	friend ostream& operator <<(ostream &os, kVec *myVec);
	float& operator[](int i);
	float operator *(kVec b); // Dot product
	kVec operator %(kVec b);  // Cross product
	kVec operator +(kVec b);
	kVec operator -(kVec b);
	kVec operator *(float b);
	kVec operator /(float b);
	kVec operator =(kVec b);
};

class kMat {
private:
	kVec * mat;
	int r; int c;
protected:
public:
	// Constructors
	kMat();
	kMat(kMat const& b);
	kMat(kVec b);
	kMat(int size);
	kMat(int rows, int cols);
	kMat(int rows, int cols, float * vals);
	~kMat();
	
	// Access
	void set(int rows, int cols, float * vals);
	kVec toVec();
	kVec getRow(int row);
	kVec getCol(int col);
	
	// Math
	float norm();
	float trace();
	kMat I();
	kMat transpose();
	int rank();
	kMat invert();
	float det();
	
	// Operators
	friend ostream& operator <<(ostream &os, const kMat &myMat);
	friend ostream& operator <<(ostream &os, kMat *myMat);
	kVec& operator[](int i);
	kVec operator *(kVec b);
	kMat operator *(kMat b);
	kMat operator *(float b);
	kMat operator +(kMat b);
	kMat operator -(kMat b);
	kMat operator /(float b);
	kMat operator =(kMat b);
	kMat operator ^(int b);
};

class kQuat {
private:
protected:
	float scl;
	float vec[3];
	friend class kTrans;
public:
	// Constructors
	kQuat();
	kQuat(const kQuat &q);
	kQuat(float x, float y, float z);
	kQuat(float s, float i, float j, float k);
	~kQuat();
	
	// Operators
	kQuat operator+ (kQuat operand);
	kQuat operator- (kQuat operand);
	kQuat operator* (float operand);
	kQuat operator* (kQuat operand);
	kQuat operator= (kQuat operand);
	
	// Methods
	kQuat conj();
	kQuat inv();
	float mag();
	void normalize();
	void scale(kQuat scaleBy);
	void unscale(kQuat scaleBy);
	void set(float w, float x, float y, float z);
	void set(kQuat newQuat);
	void print();
	float getScl() { return scl; }
	float getVecI() { return vec[0]; }
	float getVecJ() { return vec[1]; }
	float getVecK() { return vec[2]; }
	
	// Rotation
	// Note the difference between OPERATIONAL and VALUED rotational quaternion,
	// and remember that some quaternions will be merely vectors being
	// transformed by other quaternions or matrices
	void convertValuedToOperational();
	void convertOperationalToValued();
	void reverseValuedRotation();
	void reverseOperationalRotation();
};

class kTrans {
private:
	kQuat * _pos; // Position of child frame. Only i, j, k are used; scl should always be 0
	kQuat * _rot; // Rotation of child frame about parent frame. Does NOT store a rotation quaternion, but rather the strict angle-and-vector rotation values
	kQuat * _scl; // Scaling of object along local axes
protected:
public:
	kTrans();
	~kTrans();
	void setPos(float x, float y, float z);
	void addPos(float x, float y, float z);
	void setRot(float w, float x, float y, float z);
	void addRot(float w, float x, float y, float z);
	void setScl(float x, float y, float z);
	void addScl(float x, float y, float z);
	kQuat getPos();
	kQuat getRot();
	kQuat getScl();
	kTrans operator= (kTrans operand);
	kQuat applyTransformation(kQuat subject);
	kQuat reverseTransformation(kQuat subject);
	void glApply();
	void glUnapply();
	void print();
	void update(kQuat * linearVelocity, kQuat * angularVelocity, float dt);
};

namespace numericMath {
	float getTolerance();
	void setTolerance(float value);
	int getIterLimit();
	void setIterLimit(int value);
	float bisectionRoot(float(*f)(float), float low, float high);
	float bisectionSolve(float(*f)(float), float low, float high, float target);
	float fmod(float a, float b);
	int nextpoweroftwo(int x);
	//int newRound(double x);
}

// Base
class aColor {
public:
	float r;
	float g;
	float b;
	float a;
	
	aColor();
	aColor(float fr, float fg, float fb, float fa);
	void set(float fr, float fg, float fb, float fa);
	void setColor();
	void setDiffuseMat();
	void setAmbientMat();
};

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

class aTexture
{
private:
	GLuint _textureId;
	SDL_Surface * _texture;
	bool _isLoaded;
public:
	aTexture();
	~aTexture();
	bool loadFromFile(string filename);
	bool loadFromSurface(SDL_Surface * surf);
	bool unload();
	bool isLoaded() { return _isLoaded; }
	bool use();
	int getId() { return _textureId; }
};

class aObject {
private:
protected:
	kTrans * frame;
	kQuat * linearVelocity; // Defined in parent frame
	kQuat * angularVelocity; // Defined in parent frame, valued quaternion rotation
	aObject * parent;
	aObject * children;
	aObject * next;
	aColor * ambientMaterial;
	aColor * diffuseMaterial;
	aTexture * tex;
	aMesh * mesh;
public:
	// Constructors
	aObject(aObject * p = NULL);
	aObject(float x, float y, float z, aObject * p = NULL);
	aObject(kQuat orientation, aObject * p = NULL);
	aObject(float x, float y, float z, kQuat orientation, aObject * p = NULL);
	~aObject();
	void freeChildren();
	
	// Access
	kTrans getFrame();
	kQuat getLocation();
	kQuat getRotation();
	kQuat getLinearVelocity();
	kQuat getAngularVelocity();
	kQuat getScaling();
	
	// Color
	void setAmbient(float r, float g, float b);
	void setDiffuse(float r, float g, float b);
	
	void setTexture(string filename);
	void setTexture(SDL_Surface * surf);
	void setScaling(float x, float y, float z);
	// TEMPORARY, FOR DEMO; end class will allow manipulation only through physics forces
	void setPosition(float x, float y, float z);
	void setRotation(float w, float x, float y, float z);
	void setLinearVelocity(float x, float y, float z);
	void setAngularVelocity(float w, float x, float y, float z);
		
	// Methods
	virtual void update(float dt, bool updateChildren = true);
	virtual void render(bool renderChildren = true);
	
	// Child manipulation
	void addChild(aObject * newChild);
	
	// Listing
	aObject * get(int n); // n < 0 returns first; n > length returns last
	void set(aObject * value, int n); // n > length and n < 0 sets last
	int listLength();
};

class aSkybox : public aObject
{
private:
	float radius;
	int resolution;
protected:
public:
	aSkybox();
	aSkybox(float r, int n);
	~aSkybox();
	void setRadius(float r);
	void setResolution(int n);
};

class aCamera {
private:
	kQuat * position;
	kQuat * target;
	kQuat * up;
	float fieldAngle;
	float aspectRatio;
	float nearClip;
	float farClip;
	aSkybox * skybox;
protected:
public:
	aCamera();
	~aCamera();
	
	// Set accessors
	void setPosition(float x, float y, float z);
	void setTarget(float x, float y, float z);
	void setUp(float x, float y, float z);
	void setRight(float x, float y, float z);
	void setFieldAngle(float v) { fieldAngle = v; }
	void setAspectRatio(float v) { aspectRatio = v; }
	void setNearClip(float v) { nearClip = v; }
	void setFarClip(float v);

	// Camera accessors
	float getFieldAngle() { return fieldAngle; }
	float getAspectRatio() { return aspectRatio; }
	float getNearClip() { return nearClip; }
	float getFarClip() { return farClip; }
	
	// Spacial accessors
	kQuat * getPosition() { return position; }
	kQuat * getTarget() { return target; }
	kQuat * getUp() { return up; }
	
	void applyCamera();
	void print();

	// Skybox (background) settings
	void setSkyboxTexture(string filename);
	void setSkyboxTexture(SDL_Surface * surf);
	void setSkyboxResolution(int n);
	void renderSkybox();

	// Movement functions
	void rotateFocus(float dTht, float dPhi);
	void rotateCenter(float dTht, float dPhi);
	void pan(float dx, float dy, float dz);
	void focus(float x, float y, float z);
};

enum aAppState {ASTATE_GLOBAL, ASTATE_MENU, ASTATE_INGAME, ASTATE_CINEMATIC};

class aEvent {
private:
protected:
public:
	aAppState eventState;
	bool (*trigger)(void);
	void (*target)(void);
	aEvent();
	~aEvent();
};

class eventList {
private:
	aEvent * thisEvent;
	eventList * nextNode;
protected:
public:
	eventList();
	~eventList();	// NOTE: When a list is deleted, elements are also freed.
	
	// NOTE: All functions assume that they are being called for the head node.
	int listLength();
	int listLength(aAppState myState);
	aEvent * getElement(int n);
	aEvent * createElement();
	void createElement(aAppState myState, bool (*myTrigger)(void), void (*myTarget)(void));
	int createElement(aEvent * newEvent);
	void checkEvents();
	void map();
};

class aTypewriter {
private:
	TTF_Font * currFont;
	SDL_Color currColor;
	kVec currLoc;
	int _height, _width;
	int nextpoweroftwo(int x);
	
protected:
public:
	aTypewriter(int height, int width);
	~aTypewriter();
	bool setFont(char * fontName);
	bool setColor(float R, float G, float B);
	bool stamp(char * text, int x_i, int y_i);
	bool type(char * text, int x_i, int y_i);
};

class aGraphics {
private:
	int width;
	int height;
	int bpp;
	bool windowOk;
	bool isWindowed;
	SDL_Surface * screen;
	SDL_Color bgColor;
	SDL_sem * screenLock;
protected:
public:
	aTypewriter * hTypewriter;

	aGraphics();
	~aGraphics();
	bool setScreen(int newHeight, int newWidth, int newBpp);
	bool declareSettings();
	bool toggleFullscreen();
	static void printAttributes();
	void resize(int width, int height);
	void go2d();
	void go3d();
	void go3d(aCamera * cam);
	void swapBuffers();
	void clearScreen();

	// Accessors
	int getWidth() { return width; }
	int getHeight() { return height; }
	int getBpp() { return bpp; }
	bool getWindowed() { return isWindowed; }
	float getAspectRatio() { return float(width) / float(height); }

	// Test rendering
	void testLoop();
	void drawCube(float xPos, float yPos, float zPos);
	void drawTexturedCube(float xPos, float yPos, float zPos);
};

enum eKeyState {AKEY_NONE, AKEY_NUM, AKEY_CAPS, AKEY_CTRL, AKEY_SHIFT, AKEY_ALT, AKEY_COMMAND, AKEY_WINDOWS};

class aKeyboard {
private:
	bool map[312];
	bool prevMap[312];
	SDL_Event event;
protected:
public:
	aKeyboard();
	void update();
	bool checkPressDown(char key);
	bool checkLiftUp(char key);
	bool checkPrevKey(char key);
	bool checkKey(char key);
	bool checkKey(SDLKey aKey);
	eKeyState checkState();
};

class aLight {
private:
	kQuat * position; // Defined within parent frame; global frame by default
	kQuat * ambient; // Color of ambient (non-directional) light
	kQuat * diffuse; // Color of diffuse (directional) light
	aLight * next;
	float attenuation; // Light attenuation factor
	int lightId;
	bool isEnabled; // Lights are disabled by default
	bool isDirectional; // Directional lights produce parallel rays in the _position direction; non-directional lights are positional, where _position specifies the light location in the world
	bool isVisible; // Visible lights are rendered as small points with the light's ambient color
	aMesh * lightObject;
protected:
public:
	// Constructors
	aLight();
	~aLight();
	
	// Access
	void enable();
	void disable();
	bool getStatus() { return isEnabled; }
	bool getVisible() { return isVisible; }
	void setVisible(bool v) { isVisible = v; }
	
	// Lighting
	void setAmbient(float r, float g, float b);
	void setDiffuse(float r, float g, float b);
	void setPosition(float x, float y, float z);
	void setAttenuation(float a) { attenuation = a; }
	void setDirectional() { isDirectional = true; }
	void setPositional() { isDirectional = false; }
	void refreshPosition();
	
	// List methods
	aLight * get(int n); // n < 0 returns first; n > length returns last
	void set(aLight * value, int n); // n > length and n < 0 sets last
	int listLength();
	
	int getGLLightNum(int n);
};

class aModel {
private:
protected:
public:
};

class aPanel {
private:
protected:
	char * title;
	aPanel * next; // Listing
	SDL_Surface * image;
public:
	kVec pos;
	kVec dim;
	bool visible;
	aPanel();
	aPanel(int x, int y, int w, int h);
	~aPanel();
	void setTitle(char * newTitle);
	virtual void update();
	virtual void render(aGraphics * context);
	void setImage(string filename);
	void setImage(SDL_Surface * surf);
	
	// Listing
	aPanel * get(int n); // n < 0 returns first; n > length returns last
	void set(aPanel * value, int n); // n > length and n < 0 sets last
	int listLength();
};

class aStopwatch {
private:
	// Reference only: not accurate enough for timers
	time_t currTime;
	time_t lastTime;
	int currTicks;
	int lastTicks;
	tm * currTimeInfo;
	long frame;
	float dt;
	// Timers only: too high of an overhead for frequent reference usage
protected:
public:
	// Reference only: not accurate enough for timers
	aStopwatch();
	void refresh();
	float getFramerate();
	float getDt() { return dt; }
	string getDayText();
	string getMonthText();
	int getDayOfWeek();
	int getDayOfMonth();
	int getDayOfYear();
	int getHour();
	int getMinute();
	int getSecond();
	int getYear();
	string getFullTime();
	string getTimestamp();
	// Timers only: too high of an overhead for frequent reference usage
};

enum MouseState { NO_BUTTONS, LEFT_BUTTON, RIGHT_BUTTON, BOTH_BUTTONS };

class aMouse
{
private:
	int currX;
	int currY;
	int prevX;
	int prevY;
	MouseState currState;
	MouseState prevState;
protected:
public:
	aMouse(void);
	~aMouse(void);
	void update();
	void update(SDL_Event eve);

	int getCurrX() { return currX; }
	int getCurrY() { return currY; }
	int getPrevX() { return prevX; }
	int getPrevY() { return prevY; }
	int getDX() { return currX - prevX; }
	int getDY() { return currY - prevY; }
	MouseState getCurrState() { return currState; }
	MouseState getPrevState() { return prevState; }
};

class aApp {
private:
protected:
	aPanel * panels;
	aObject * origin;
	aLight * lights;
	kQuat * globalAmbient;
public:
	// Data
	aGraphics * hGraphics;
	aStopwatch * hStopwatch;
	aKeyboard * hKeyboard;
	aMouse * hMouse;
	aAppState currState;
	eventList * gameEvents;
	aCamera * camera;
	bool debugging;
	bool isLooping;
	bool (*externalRender)(void);
	ofstream debugFile;
	
	// Structors
	aApp();
	~aApp();
	
	// Functions
	void initialize(int height, int width);
	void execute();
	void terminate();
	void mainLoop();
	void update();
	void render();
	void debug(char * str);
	
	// List management
	void addPanel(aPanel * p);
	void addObject(aObject * o);
	void addLight(aLight * l);
	
	// Global settings
	void setGlobalAmbient(float r, float g, float b);
};

// Implementation

class cLine {
private:
protected:
public:
	char * contents;
	cLine * next;
	int nToEnd;
	cLine() {
		contents = NULL;
		next = NULL;
		nToEnd = 0;
	}
	cLine(char * stuff) {
		contents = stuff;
		next = NULL;
		nToEnd = 0;
	}
	int getLineLength() {
		int toReturn = 0;
		for (int i = 0; contents[i] != NULL; i++) {
			toReturn++;
		}
		return toReturn;
	};
};

class aConsole : public aPanel {
private:
	cLine * bottomLine;
protected:
public:
	aConsole();
	~aConsole();
	void addLine(char * contents);
	char * getLine(int n);
	void render();
};

class aCube : public aObject {
private:
	float size;
protected:
public:
	// Structors
	aCube();
	virtual ~aCube();

	// Accessors
	float getSize() { return size; }
	void setSize(float v);
};

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

#endif