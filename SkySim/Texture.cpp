
#include "Texture.h"

Texture::Texture()
{
	aTexture = NULL;
	Width = 0;
	Height = 0;
}
Texture::~Texture()
{
	Free();
}

//Create blank texture, of the passed width and height, with streaming access
void Texture::Create(int width, int height, SDL_Renderer* rend, SDL_Window* wind)
{
	aTexture = SDL_CreateTexture(rend, SDL_GetWindowPixelFormat(wind), SDL_TEXTUREACCESS_STREAMING, width, height);
	Width = width;
	Height = height;
}

//Destroys texture and resets relevant variables
void Texture::Free()
{
	if (aTexture != NULL)
	{
		SDL_DestroyTexture(aTexture);
		aTexture = NULL;
		Width = 0;
		Height = 0;
	}
}

//Set the pixels of the texture to the RGB data contained in the passed array
void Texture::SetPixels(std::shared_ptr<Pixel> image, SDL_PixelFormat * pixform, int byte_width)
{
	void* Pixels;
	// provide access to pixels of atexture at the address of pixels
	SDL_LockTexture(aTexture, NULL, &Pixels, &byte_width);

	//Cast generic pointer of texture pixels to Uint32 so that the memory can be set normally
	Uint32 *Ptr = static_cast<Uint32*>(Pixels);
	for (int i = 0; i < Width * Height; i++)
	{
		Ptr[i] = SDL_MapRGB(pixform, image.get()[i].Red, image.get()[i].Green, image.get()[i].Blue);
	}

	//unlock texture so that it can be displayed
	SDL_UnlockTexture(aTexture);
	Pixels = NULL;
}

//Return a pointer to the actual SDL_texture so that it can be rendered
SDL_Texture* Texture::GetTexture()
{
	return aTexture;
}
