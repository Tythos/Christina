/*
 *  smartList.cpp
 *  sandbox
 *
 *  Created by Brian Kirkpatrick on 2/9/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include <iostream>
#include "smartList.h"

using namespace std;

// --- Constructors --- //
template <class T> smartList<T>::smartList() {
	// Default constructor
	pItem = NULL;
	pNext = NULL;
	nToEnd = 0;
}

template <class T> smartList<T>::smartList(const smartList &rhs) {
	smartList<T> * pTmp = this;
	int l = rhs.length();
	for (int i = 0; i < l; i++) {
		(*pTmp->pItem) = (*rhs.get(i));
		pTmp->pNext = new smartList<T>();
		pTmp->nToEnd = l - i - 1;
		pTmp = pTmp->pNext;
	}
}

template <class T> smartList<T>::~smartList() {
	// Destructor
	if (pNext != NULL) delete pNext;
}

// --- Accessors --- //
template <class T> T * smartList<T>::get(int n, int l) {
	// Private accessor; compared known length to nToEnd
	if (nToEnd == l - n - 1) return pItem;
	else if (pNext == NULL) return NULL;
	else return pNext->get(n, l);
}

template <class T> T * smartList<T>::get(int n) {
	// Public accessor (should only be used from head node of smartList)
	n = n % length();
	return this->get(n, nToEnd + 1);
}

template <class T> int smartList<T>::length() {
	// Length of list is nodes to end of list, plus one
	if (pItem == NULL) return 0;
	else return nToEnd + 1;
}

template <class T> void smartList<T>::add(T * newItem) {
	// Add new nodes as head node, shift old head down one
	if (pItem == NULL) {
		// Add first
		pItem = newItem;
		nToEnd = 0;
	} else {
		// Add as first, point to second
		smartList<T> * newNode = new smartList<T>();
		newNode->pItem = pItem;
		newNode->nToEnd = nToEnd;
		newNode->pNext = pNext;
		pItem = newItem;
		nToEnd++;
		pNext = newNode;
	}
}

template <class T> void smartList<T>::remove(int n) {
	int l = length();
	smartList<T> * currNode = this;
	smartList<T> * prevNode = NULL;
	if (n >= l || n < 0) return;
	// Removes the nth element from this list and adjusts numbering
	if (n == 0) {
		// To remove the first (pointer) node, copy then delete the second node
		if (l == 1) {
			// If there is only one node, delete it
			pItem = NULL;
			nToEnd = 0;
			return;
		}
		smartList<T> * toDelete = pNext;
		pItem = pNext->pItem;
		nToEnd = nToEnd - 1;
		pNext = pNext->pNext;
		delete toDelete;
		return;
	}
	while(n != l - currNode->nToEnd - 1) {
		currNode->nToEnd = currNode->nToEnd - 1;
		prevNode = currNode;
		currNode = currNode->pNext;
	}
	if (prevNode != NULL) prevNode->pNext = currNode->pNext;
	currNode->pItem = NULL;
	delete currNode;
}

// --- Operators --- //
template <class T> T& smartList<T>::operator[](int offset) {
	T * toReturn = this->get(offset);
	return *toReturn;
}

template <class T> void smartList<T>::print() {
	print(length());
	return;
}

template <class T> void smartList<T>::print(int l) {
	cout << "Item " << l - nToEnd - 1 << ": " << (*pItem) << endl;
	if (pNext != NULL) pNext->print(l);
	return;
}