/*
 *  numericMath.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/3/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

// Necessary for Win32 math.h to expose math constant definitions (M_PI, etc)
#define _USE_MATH_DEFINES
#include <math.h>

#include <stdio.h>
#include "numericMath.h"

namespace numericMath {
	int gIterLimit = 0;
	float gTolerance = 0.001f;
	
	float bisectionRoot(float(*f)(float), float low, float high) {
		// Solve for the root of function f via bisection from low, high
		float mid = (low + high) / 2.0f;
		int iter = 0;
		while (f(low) * f(high) < 0 && fabs(f(mid)) > gTolerance) {
			mid = (low + high) / 2.0f;
			if (f(low) * f(mid) < 0) high = mid;
			else if (f(high) * f(mid) < 0) low = mid;
			iter++;
		}
		return mid;
	}
	
	float bisectionSolve(float(*f)(float), float low, float high, float target) {
		// Solve for the root of function f via bisection from low, high
		float mid = (low + high) / 2.0f;
		int iter = 0;
		while ((f(low)-target) * (f(high)-target) < 0 && fabs(f(mid)-target) > gTolerance && iter < gIterLimit) {
			mid = (low + high) / 2.0f;
			if ((f(low)-target) * (f(mid)-target) < 0) high = mid;
			else if ((f(high)-target) * (f(mid)-target) < 0) low = mid;
			iter++;
		}
		if (iter == gIterLimit)
			printf("Numerical Error: Iteration limit reached w/o convergence via bisection\n");
		return mid;
	}
	
	float getTolerance() {
		return gTolerance;
	}
	
	void setTolerance(float value) {
		gTolerance = value;
	}
	
	int getIterLimit() {
		return gIterLimit;
	}
	
	void setIterLimit(int value) {
		gIterLimit = value;
	}

	float fmod(float a, float b)
	{
		if (b == 0) return a;
		int result = static_cast<int>( a / b );
		return a - static_cast<float>( result ) * b;
	}
	
	int nextpoweroftwo(int x) {
		float arg = (float)x;
		double logbase2 = log(arg) / log(2.0f);
		double d = pow(2, ceil(logbase2));
		return (int)(d + 0.5);
	}	
}