#ifndef ARTEMIS_CURSOR_H
#define ARTEMIS_CURSOR_H

#include "kString.h"
#include "SDL.h"
#include "aGraphics.h"
#include "aMouse.h"
#include "aPanel.h"

using namespace std;

class aCursor : public aPanel {
private:
	aMouse * mouse;
protected:
public:
	aCursor(aMouse * m);
	~aCursor(void);
	void render(aGraphics * context);
};

#endif