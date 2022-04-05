/*
 *  libArtemis.h
 *  Static library header for external usage
 *
 *  Created by Brian Kirkpatrick on 10/10/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef LIB_ARTEMIS_H
#define LIB_ARTEMIS_H

// Libraries
#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <SDL_image.h>
#include <SDL_mixer.h>

// Utility
#include "Conversion.h"
#include "DataStructures.h"

// Math
#include "kMat.h"
#include "kQuat.h"
#include "kTrans.h"
#include "kVec.h"
#include "numericMath.h"

// Base
#include "aApp.h"
#include "aCamera.h"
#include "aEvent.h"
#include "aGraphics.h"
#include "aKeyboard.h"
#include "aLight.h"
#include "aMesh.h"
#include "aModel.h"
#include "aObject.h"
#include "aPanel.h"
#include "aStopwatch.h"
#include "aTypewriter.h"
#include "aSkybox.h"

// Implementation
#include "aConsole.h"
#include "aCube.h"
#include "aPlanet.h"
#include "aButton.h"
#include "aCursor.h"
#include "aGraph.h"

#endif