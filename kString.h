#ifndef KIRK_STRING_H
#define KIRK_STRING_H

#include <stdio.h>

class kString
{
private:
	char * string;
	int length;
	bool isInit;
protected:
public:
	// Constructors
	kString();
	kString(const char * s);
	kString(const kString& s);
	kString(const int i);
	kString(const float f);
	~kString();

	// Accessors
	int getLength() { return length; }
	char operator[](int n);
	const char operator[](int n) const;
	char * raw();

	// Assignment
	kString& operator=(const kString &rhs);
	kString& operator=(const char* rhs);

	// Concatenation
	kString operator+=(const kString &rhs);
	const kString operator+(const kString &rhs);

	// Comparison
	bool operator==(const kString &rhs) const;
	bool operator!=(const kString &rhs) const;
};

#endif