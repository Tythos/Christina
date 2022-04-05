/*
 *  kMat.cpp
 *  kMat
 *
 *  Created by Brian Kirkpatrick on 2/20/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "kMat.h"
#define SMALL_FLOAT 0.0001f

// --- Constructors --- //
kMat::kMat():
mat(NULL), r(0), c(0) {
}

kMat::kMat(kMat const& b) {
	r = b.r; c = b.c;
	mat = new kVec[r];
	for (int i = 0; i < r; i++) {
		mat[i] = b.mat[i];
	}
}

kMat::kMat(kVec b) {
	// Creates a 1 x n matrix from given vector
	r = 1; c = b.getLength();
	mat = new kVec[1];
	mat[0] = b;
}

kMat::kMat(int size) {
	r = size; c = size;
	mat = new kVec[r];
	const int cc = c;
	float * tmpRow = new float[cc];
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			if (i == j) tmpRow[j] = 1.0f;
			else tmpRow[j] = 0.0f;
		}
		mat[i] = kVec(c, tmpRow);
	}
	delete[] tmpRow;
}

kMat::kMat(int rows, int cols) {
	r = rows; c = cols;
	mat = new kVec[r];
	const int cc = c;
	float * tmpRow = new float[cc];
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			if (i == j) tmpRow[j] = 1.0f;
			else tmpRow[j] = 0.0f;
		}
		mat[i] = kVec(c, tmpRow);
	}
	delete[] tmpRow;
}

kMat::kMat(int rows, int cols, float * vals) {
	r = rows; c = cols;
	mat = new kVec[r];
	float * tmpRow = new float[c];
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			tmpRow[j] = vals[i*c+j];
		}
		mat[i] = kVec(c, tmpRow);
	}
	delete[] tmpRow;
}

kMat::~kMat() {
	if (mat != NULL) delete [] mat;
}

// --- Access --- //
void kMat::set(int rows, int cols, float * vals=NULL) {
	r = rows; c = cols;
	if (mat != NULL) delete mat;
	mat = new kVec[r];
	const int cc = c;
	float * tmpRow = new float[cc];
	if (vals != NULL) {
		for (int i = 0; i < r; i++) {
			for (int j = 0; j < c; j++) {
				tmpRow[j] = vals[i*c+j];
			}
			mat[i] = kVec(c, tmpRow);
		}
	} else {
		for (int i = 0; i < r; i++) {
			for (int j = 0; j < c; j++) {
				tmpRow[j] = (i == j) ? 1.0f : 0.0f;
			}
			mat[i] = kVec(c, tmpRow);
		}
	}
}

kVec kMat::toVec() {
	// If matrix dimensions are 1 x n or n x 1, returns the equivalent vector
	if (r == 1) {
		kVec * toRet = new kVec(c);
		for (int i = 0; i < c; i++) {
			(*toRet)[i] = (*this)[1][i];
		}
		return (*toRet);
	} else if (c == 1) {
		kVec * toRet = new kVec(r);
		for (int i = 0; i < r; i++) {
			(*toRet)[i] = (*this)[i][1];
		}
		return (*toRet);
	} else {
		cout << "Exception 5 in kMat::toVec: 1d matrix required\n";
		throw 5;
	}
}

kVec kMat::getRow(int row) {
	return mat[row];
}

kVec kMat::getCol(int col) {
	kVec * toRet = new kVec(r);
	for (int i = 0; i < r; i++) {
		(*toRet)[i] = mat[i][col];
	}
	return (*toRet);
}

// --- Math --- //
float kMat::norm() {
	// return sqrt(max eig of A_H * A)
	return 0.0f;
}

float kMat::trace() {
	float toRet = 0.0f;
	if (r != c) {
		cout << "Exception 3 in kMat::trace: square matrix required\n" << endl;
		throw 3;
	} else {
		for (int i = 0; i < r; i++) {
			toRet += (mat[i])[i];
		}
	}
	return toRet;
}

kMat kMat::I() {
	if (r != c) {
		cout << "Exception 3 in kMat::identity: square matrix required\n";
		throw 3;
	}
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			mat[i][j] = i == j ? 1.0f : 0.0f;
		}
	}
	return (*this);
}

kMat kMat::transpose() {
	const int cc = r * c;
	float * myVals = new float[cc];
	for (int i = 0; i < c; i++) {
		for (int j = 0; j < r; j++) {
			myVals[i*r+j] = mat[j][i];
		}
	}
	kMat * toRet = new kMat(c, r, myVals);
	return (*toRet);
}

kMat kMat::invert() {
	float d = det();
	if (d*d < SMALL_FLOAT*SMALL_FLOAT) {
		cout << "Exception 6 in kMat::invert: insufficient rank for operation\n";
		throw 6;
	}
	return (*this);
}

float kMat::det() {
	if (r != c) {
		cout << "Exception 3 in kMat::operator^: square matrix required\n";
		throw 3;
	}
	float toRet = 0.0f;
	if (r == 2) {
		toRet = (*this)[0][0] * (*this)[1][1] - (*this)[0][1] * (*this)[1][0];
		return toRet;
	}
	kMat * subMat = new kMat(r-1, c-1);
	for (int j = 0; j < c; j++) {
		for (int si = 1; si < r; si++) {
			for (int sj = j+1; sj < j+c; sj++) {
				(*subMat)[si-1][sj-j-1] = (*this)[si][sj];
			}
		}
		toRet += mat[0][j] * subMat->det();
	}
	return toRet;
}

// --- Operators ---//
ostream& operator <<(ostream &os, const kMat &myMat) {
	os << endl;
	for (int i = 0; i < myMat.r; i++) {
		if (i == 0 || i == myMat.r - 1) os << "+";
		else os << "|";
		os << myMat.mat[i];
		if (i == 0 || i == myMat.r - 1) os << "+";
		else os << "|" << endl;
		if (i == 0) os << endl;
	}
	return os;
}

ostream& operator <<(ostream &os, kMat *myMat) {
	os << (*myMat);
	return os;
}

kVec& kMat::operator[](int i) {
	if (i < 0) {
		cout << "Exception 1 in kMat::operator[]: invalid index\n";
		throw 1;
	}
	if (r == 0 || c == 0 || mat == NULL) {
		cout << "Exception 4 in kMat::operator[]: matrix not initialized\n";
		throw 4;
	}
	i = i % r;
	return mat[i];
}

kVec kMat::operator *(kVec b) {
	if (b.getLength() != c) {
		cout << "Exception 2 in kMat::*(kVec): incompatible dimensions\n";
		throw 2;
	}
	kMat * eqMat = new kMat(b);
	return ((*this) * eqMat->transpose()).toVec();
}

kMat kMat::operator *(kMat b) {
	if (c != b.r) {
		cout << "Exception 2 in kMat::*(kMat): incompatible dimensions\n";
		throw 2;
	}
	kMat * toRet = new kMat(r, b.c);
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < b.c; j++) {
			(*toRet)[i][j] = mat[i] * b.getCol(i);
		}
	}
	return (*toRet);
}

kMat kMat::operator *(float b) {
	kMat * toRet = new kMat(r, c);
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			(*toRet)[i][j] = mat[i][j] * b;
		}
	}
	return (*toRet);
}

kMat kMat::operator +(kMat b) {
	if (c != b.c || r != b.r) {
		cout << "Exception 2 in kMat::+(kMat): incompatible dimensions\n";
		throw 2;
	}
	kMat * toRet = new kMat(r,c);
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			(*toRet)[i][j] = mat[i][j] + b[i][j];
		}
	}
	return (*toRet);
}

kMat kMat::operator -(kMat b) {
	return (*this) + (b * -1);
}

kMat kMat::operator /(float b) {
	return (*this) * (1 / b);
}

kMat kMat::operator =(kMat b) {
	if (mat != NULL) delete [] mat;
	r = b.r; c = b.c;
	mat = new kVec[r];
	for (int i = 0; i < r; i++) {
		mat[i] = b.mat[i];
	}
	return (*this);
}

kMat kMat::operator ^(int b) {
	if (c != r) {
		cout << "Exception 3 in kMat::operator^: square matrix required\n";
		throw 3;
	}
	kMat * toRet = new kMat(r, c);
	if (b == 0) return toRet->I();
	(*toRet) = (*this);
	for (int i = 1; i < abs(b); i++) {
		(*toRet) = (*toRet) * (*this);
	}
	if (b < 0) (*toRet) = toRet->invert();
	return (*toRet);
}