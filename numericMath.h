/*
 *  numericMath.h
 *  superFluids
 *
 *  Created by Brian Kirkpatrick on 2/4/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 *  Contains tools for finding numerical solutions to functions
 */

#ifndef NUMERIC_MATH_H
#define NUMERIC_MATH_H

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

#endif