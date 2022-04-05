/*
 *  aEvent.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/23/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_EVENT_H
#define ARTEMIS_EVENT_H

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

#endif