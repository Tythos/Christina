#include "aCursor.h"

aCursor::aCursor(aMouse * m)
: aPanel(0.0f, 0.0f, 16.0f, 16.0f) {
	SDL_ShowCursor(SDL_DISABLE);
	mouse = m;
	x.unit = UNIT_PCT;
	y.unit = UNIT_PCT;
	w.align = ALIGN_NEGATIVE;
	h.align = ALIGN_POSITIVE;
	isTranslucent = true;
}

aCursor::~aCursor(void) {
	// Do not delete mouse; linked, not generated
}

void aCursor::render(aGraphics * context) {
	x.value = mouse->getXCoord().toPct((float)(context->getWidth()));
	y.value = 1.0f - mouse->getYCoord().toPct((float)(context->getHeight()));
	aPanel::render(context);
}