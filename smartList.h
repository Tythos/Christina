/*
 *  smartList.h
 *  sandbox
 *
 *  Created by Brian Kirkpatrick on 2/9/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef SMARTLIST_H
#define SMARTLIST_H

template <class T> class smartList {
private:
	T * pItem;
	smartList * pNext;
	int nToEnd;
	T * get(int n, int length);
	void print(int l);
protected:
public:
	// Constructors
	smartList();
	smartList(const smartList<T> &rhs);
	~smartList();
	
	// Accessors
	T * get(int n);
	int length();
	void add(T * newItem);
	void remove(int n);
	
	// Operators
	void print();
	T& operator[](int offset);
};

#endif