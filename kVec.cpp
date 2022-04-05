/*
 *  kVec.cpp
 *  kVec
 *
 *  Created by Brian Kirkpatrick on 2/20/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "kVec.h"
#include <math.h>

// --- Constructors --- //
kVec::kVec() {
	len = 0;
	vec = NULL;
}

kVec::kVec(int length) {
	len = length;
	vec = new float[len];
}

kVec::kVec(int length, float vals[]) {
	len = length;
	vec = new float[len];
	for (int i = 0; i < length; i++) {
		vec[i] = vals[i];
	}
}

kVec::kVec(kVec const& b) {
	len = b.len;
	vec = new float[len];
	for (int i = 0; i < len; i++) {
		vec[i] = b.vec[i];
	}
}

kVec::~kVec() {
	delete vec;
}

// --- Access --- //
void kVec::set(int length, float vals[]) {
	if (vec != NULL) delete vec;
	len = length;
	vec = new float[len];
	for (int i = 0; i < len; i++) {
		vec[i] = vals[i];
	}
}

// --- Math --- //
float kVec::abs() {
	float toRet = 0.0;
	for (int i = 0; i < len; i++) {
		toRet += vec[i] * vec[i];
	}
	return sqrt(toRet);
}

// --- Operators --- //
ostream & operator << (ostream &os, const kVec &myVec) {
	int i = 0;
	os << "[";
	for (i = 0; i < myVec.len-1; i++) {		
		os << myVec.vec[i] << ", ";
	}
	if (myVec.len > 0) os << myVec.vec[myVec.len-1];
	os << "]";
	return os;
}

ostream& operator <<(ostream &os, kVec *myVec) {
	os << (*myVec);
	return os;
}

float& kVec::operator[](int i) {
	if (i < 0) {
		cout << "Exception 1 in operator[]: invalid index\n";
		throw 1;
	}
	i = len > 0 ? i % len : i;
	return vec[i];
}

float kVec::operator *(kVec b) { // Dot product
	float toRet = 0.0;
	int minLen = this->len < b.len ? this->len : b.len;
	for (int i = 0; i < minLen; i++) {
		toRet += this->vec[i] * b.vec[i];
	}
	return toRet;
}

kVec kVec::operator %(kVec b) {  // Cross product
	if (this->len != b.len) {
		cout << "Exception 2 in operator %: incompatible dimensions\n";
		throw 2;
	}
	const int l = len;
	float * newVec = new float[l];
	for (int i = 0; i < len; i++) {
		newVec[i] = vec[i+1] * b[i+2] - b[i+1] * vec[i+2];
	}
	kVec * toRet = new kVec(len, newVec);
	return (*toRet);
}

kVec kVec::operator +(kVec b) {
	if (this->len != b.len) {
		cout << "Exception 2 in operator +: incompatible dimensions\n";
		throw 2;
	}
	const int l = len;
	float * newVec = new float[len];
	for (int i = 0; i < len; i++) {
		newVec[i] = vec[i] + b[i];
	}
	kVec * toRet = new kVec(len, newVec);
	return (*toRet);
}

kVec kVec::operator -(kVec b) {
	return (*this) + (b * -1);
}

kVec kVec::operator *(float b) {
	const int l = len;
	float * newVec = new float[l];
	for (int i = 0; i < len; i++) {
		newVec[i] = vec[i] * b;
	}
	kVec * toRet = new kVec(len, newVec);
	return (*toRet);
}

kVec kVec::operator /(float b) {
	return (*this) * (1 / b);
}

kVec kVec::operator =(kVec b) {
	if (vec != NULL) delete vec;
	len = b.len;
	vec = new float[len];
	for (int i = 0; i < len; i++) {
		vec[i] = b[i];
	}
	return (*this);
}