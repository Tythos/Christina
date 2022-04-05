/*
 *  aSoundboard.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 4/26/10.
 *  Copyright 2010 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_SOUNDBOARD_H
#define ARTEMIS_SOUNDBOARD_H

#include "SDL_mixer.h"
#include "kString.h"

class aSoundboard {
private:
	bool isInit;
	int audioRate;
	int audioFormat;
	int numChannels;
	int bufferSize;
	int fadeLength; // Length of fade effects, in ms
	float volume;
protected:
public:
	aSoundboard();
	~aSoundboard();

	// Sound effects
	Mix_Chunk * loadSound(kString filename);
	void playSound(kString filename);
	void playSound(Mix_Chunk * sound);

	// Music songs
	Mix_Music * loadSong(kString filename);
	void playSong(kString filename);
	void playSong(Mix_Music * mus);
	void stopSong();
	bool isPlaying();
	bool repeatSongs;
};

#endif