/*
 *  aStopwatch.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/8/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_STOPWATCH_H
#define ARTEMIS_STOPWATCH_H

#include <ctime>
#include "kString.h"

using namespace std;

class aStopwatch {
private:
	// Reference only: not accurate enough for timers
	time_t currTime;
	time_t lastTime;
	int currTicks;
	int lastTicks;
	tm currTimeInfo;
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
	kString getDayText();
	kString getMonthText();
	int getDayOfWeek();
	int getDayOfMonth();
	int getDayOfYear();
	int getHour();
	int getMinute();
	int getSecond();
	int getYear();
	kString getFullTime();
	kString getTimestamp();
	// Timers only: too high of an overhead for frequent reference usage
};

#endif