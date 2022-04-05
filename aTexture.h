#ifndef ARTEMIS_TEXTURE_H
#define ARTEMIS_TEXTURE_H

#include "kString.h"
#include "SDL.h"
#include <SDL_opengl.h>
#include "SDL_image.h"

class aTexture
{
private:
	GLuint _textureId;
	SDL_Surface * _texture;
	bool _isLoaded;
public:
	aTexture();
	~aTexture();
	bool loadFromFile(kString filename);
	bool loadFromSurface(SDL_Surface * surf);
	bool unload();
	bool isLoaded() { return _isLoaded; }
	bool bind();
	bool unbind();
	int getId() { return _textureId; }
};

#endif