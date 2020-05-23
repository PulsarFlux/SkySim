#pragma once
#include "Sky.h"

class Texture
{
public:
	Texture();
	~Texture();

	//Create blank texture, of the passed width and height, with streaming access
	void Create(int, int, SDL_Renderer*, SDL_Window*);

	//Destroys texture and resets relevant variables
	void Free();

	//Set the pixels of the texture to the RGB data contained in the passed array
	void SetPixels(std::shared_ptr<Pixel>, SDL_PixelFormat *, int);

	//Return a pointer to the actual SDL_texture so that it can be rendered
	SDL_Texture* GetTexture();

private:
	//The actual texture
	SDL_Texture* aTexture;

	//Image dimensions
	int Width;
	int Height;
};