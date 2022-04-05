/*
 *  aStopwatch.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/8/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aStopwatch.h"
#include "SDL.h"

using namespace std;

aStopwatch::aStopwatch() {
	this->refresh();
	currTicks = 0;
	frame = 0;
	dt = 1.0f;
}

void aStopwatch::refresh() {
	// Only call ONCE PER CYCLE
	this->lastTime = this->currTime;
	lastTicks = currTicks;
	do {
		time(&currTime);
		localtime_s(&currTimeInfo, &currTime);
		currTicks = SDL_GetTicks();
		dt = (currTicks - lastTicks) / 1000.0f;
	} while (dt == 0.0f);
	frame++;
}

float aStopwatch::getFramerate() {
//	printf("frame: %li, framerate: %f fps\n", frame, 1.0f / dt);
	return 1.0f / dt;
}

kString aStopwatch::getDayText() {
	return "Monday";
}

kString aStopwatch::getMonthText() {
	return "February";
}

int aStopwatch::getDayOfWeek() {
	return this->currTimeInfo.tm_wday;
}

int aStopwatch::getDayOfMonth() {
	return this->currTimeInfo.tm_mday;
}

int aStopwatch::getDayOfYear() {
	return this->currTimeInfo.tm_yday;
}

int aStopwatch::getHour() {
	return this->currTimeInfo.tm_hour;
}

int aStopwatch::getMinute() {
	return this->currTimeInfo.tm_min;
}

int aStopwatch::getSecond() {
	return this->currTimeInfo.tm_sec;
}

int aStopwatch::getYear() {
	return this->currTimeInfo.tm_year;
}

kString aStopwatch::getFullTime() {
	kString toReturn("[fu:ll:tm]");
	return toReturn;
}

kString aStopwatch::getTimestamp() {
	kString toReturn = kString("[") + kString(currTimeInfo.tm_hour);
	toReturn = toReturn + kString(":");
	toReturn = toReturn + kString(currTimeInfo.tm_min);
	toReturn = toReturn + kString(":");
	toReturn = toReturn + kString(currTimeInfo.tm_sec);
	toReturn = toReturn + kString("]");
	return toReturn;
}