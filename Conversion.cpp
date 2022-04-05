/*
 *  Conversion.cpp
 *  Conversion
 *
 *  Created by Brian Kirkpatrick on 10/2/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include <sstream>
#include "Conversion.h"

char * Conversion::floatToStr(float myFloat) {
	// Note: Only use this function once per stream call, or the output will be duplicated
	std::stringstream output;
	output << myFloat;
	std::string s = output.str();
	return (char*)s.c_str();
}

char * Conversion::intToStr(int myInt) {
	// Note: Only use this function once per stream call, or the output will be duplicated
	std::stringstream output;
	output << myInt;
	std::string s = output.str();
	return (char*)s.c_str();
}
