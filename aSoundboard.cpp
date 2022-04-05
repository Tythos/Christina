/*
 *  aSoundboard.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 4/26/10.
 *  Copyright 2010 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aSoundboard.h"

aSoundboard::aSoundboard() {
	// Initialize SDL mixer
	audioRate = 22050;
	audioFormat = AUDIO_S16SYS;
	numChannels = 2;
	bufferSize = 4096;
	fadeLength = 1000;
	volume = 1.0f;
	repeatSongs = true;
	if (Mix_OpenAudio(audioRate, audioFormat, numChannels, bufferSize) != 0) {
		isInit = false;
	} else {
		isInit = true;
	}
}

aSoundboard::~aSoundboard() {
	if (isInit) Mix_CloseAudio();
}

Mix_Chunk * aSoundboard::loadSound(kString filename) {
	Mix_Chunk * toReturn = Mix_LoadWAV(filename.raw());
	return toReturn;
}

void aSoundboard::playSound(kString filename) {
	Mix_Chunk * snd = loadSound(filename);
	playSound(snd);
}

void aSoundboard::playSound(Mix_Chunk * sound) {
	int channel = Mix_PlayChannel(-1, sound, 0);
}

Mix_Music * aSoundboard::loadSong(kString filename) {
	Mix_Music * toReturn = Mix_LoadMUS(filename.raw());
	return toReturn;
}

void aSoundboard::playSong(kString filename) {
	Mix_Music * mus = loadSong(filename);
	playSong(mus);
}

void aSoundboard::playSong(Mix_Music * mus) {
	if (repeatSongs) {
		Mix_FadeInMusic(mus, -1, fadeLength);
	} else {
		Mix_FadeInMusic(mus, 0, fadeLength);
	}
}

void aSoundboard::stopSong() {
	Mix_FadeOutMusic(fadeLength);
}

bool aSoundboard::isPlaying() {
	return Mix_PlayingMusic() == 1;
}