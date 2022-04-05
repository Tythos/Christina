/*
 *  aEvent.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/23/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aEvent.h"
#include <stdio.h>

bool blankTrigger() {
	return false;
}

void blankTarget() {
	return;
}

// --- aEvent class --- //
aEvent::aEvent() {
	this->trigger = (*blankTrigger);
	this->target = (*blankTarget);
}

aEvent::~aEvent() {
}

// --- eventList class --- //
// Manages linked lists of large numbers of events.
eventList::eventList() {
	this->thisEvent = NULL;
	this->nextNode = NULL;
}

eventList::~eventList() {
	if (this->nextNode != NULL)
		delete this->nextNode;
	if (this->thisEvent != NULL)
		delete this->thisEvent;
}

int eventList::listLength() {
	// Returns the length of this list.
	eventList * currentNode = this;
	int lengthCounter = 0;
	while(currentNode->thisEvent != NULL && currentNode->nextNode != NULL) {
		lengthCounter++;
		currentNode = currentNode->nextNode;
	}
	return lengthCounter;
}

aEvent * eventList::getElement(int n) {
	// Returns the pointer to the nth object (NOT the list node).
	int maxLength = this->listLength();
	eventList * currentNode = this;
	if (n >= maxLength) {
		n = maxLength - 1;
	}
	
	for (int i = 0; i < n; i++) {
		currentNode = currentNode->nextNode;
	}
	return currentNode->thisEvent;
}

aEvent * eventList::createElement() {
	// Creates a new object of the base type (aObject); returns it's location
	eventList * currentNode = this;
	while(currentNode->nextNode != NULL && currentNode->thisEvent != NULL) {
		currentNode = currentNode->nextNode;
	}
	currentNode->nextNode = new eventList;
	currentNode->thisEvent = new aEvent;
	return currentNode->thisEvent;
}

int eventList::createElement(aEvent * newEvent) {
	// Creates a new element in the array whose event is in the passed location
	// The index of the new element is returned.
	if (this->thisEvent == NULL) {
		this->thisEvent = newEvent;
		return 0;
	}
	int maxLength = this->listLength();
	eventList * currentNode = this;
	while(currentNode->nextNode != NULL && this->thisEvent != NULL) {
		currentNode = currentNode->nextNode;
	}
	currentNode->nextNode = new eventList;
	currentNode = currentNode->nextNode;
	currentNode->nextNode = NULL;
	currentNode->thisEvent = newEvent;
	return maxLength;
}

void eventList::createElement(aAppState myState, bool (*myTrigger)(void), void (*myTarget)(void)) {
	static int nodesCreated = 0;
	eventList * currentNode = this;
	while(currentNode->nextNode != NULL && currentNode->thisEvent != NULL) {
		currentNode = currentNode->nextNode;
	}
	nodesCreated++;
	currentNode->nextNode = new eventList();
	currentNode->thisEvent = new aEvent;
	currentNode->thisEvent->eventState = myState;
	currentNode->thisEvent->trigger = myTrigger;
	currentNode->thisEvent->target = myTarget;
}

void eventList::checkEvents() {
	// Check all events for the current state
	aEvent * eventPointer;
	for (int i = 0; i < this->listLength(); i++) {
		eventPointer = this->getElement(i);
		if (eventPointer->trigger()) {
			eventPointer->target();
		}
	}
}

void eventList::map() {
	eventList * currentNode = this;
	char * te = (char*)"";
	char * nn = (char*)"";
	for (int i = 0; i < 10; i++) {
		if (currentNode->thisEvent == NULL)
			te = (char*)"NULL";
		else
			te = (char*)"EVENT";
		if (currentNode->nextNode == NULL)
			nn = (char*)"NULL";
		else
			nn = (char*)"NODE";
		printf("NODE %i: thisEvent, %s; nextNode, %s\n", i, te, nn);
		if (currentNode->thisEvent == NULL && currentNode->nextNode == NULL)
			break;
		else
			currentNode = currentNode->nextNode;
	}
}