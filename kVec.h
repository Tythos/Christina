/*
 *  kVec.h
 *  kVec
 *
 *  Created by Brian Kirkpatrick on 2/20/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 *  Exceptions:
 *    1 - invalid index
 *    2 - incompatible dimensions
 */

#ifndef K_VECTOR_H
#define K_VECTOR_H

#include <iostream>
using namespace std;

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

#endif