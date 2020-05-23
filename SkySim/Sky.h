#pragma once
#include "Vectors.h"
#include "Physics.h"
#include <memory>
#include <SDL.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

//struct for storing RGB values to be displayed on screen
struct Pixel
{
	Uint8 Red;
	Uint8 Green;
	Uint8 Blue;
};
//struct for storing RGB values from the simulation before they are mapped to screen values
struct Pixel_Unmapped
{
	double Red;
	double Green;
	double Blue;
};

class Simulator
{
public:
	Simulator(Variables);

	//given the position of the observer, the direction they are looking in, and the direction of the sun the look of the sky can be computed
	//the position of the observer must be within the atmosphere
	std::shared_ptr<Pixel> ProduceImage(SphVec, SphVec, SphVec);

private:
	//Peforms generalised calculations
	PhysicsEngine Engine;
	class Cloud** Clouds;

	//FOV here specified in degrees
	const double FOV;

	//Screen dimension constants
	const int SCREEN_WIDTH;
	const int SCREEN_HEIGHT;

	//all distance values are specified in kilometres
	const double EARTH_RADIUS;
	const double ATMO_RADIUS;

	const double SUN_ANGULAR_RADIUS = 2 * (M_PI / 180);

	//scaling factor for most pixels
	const double SUN_INTENSITY;
	//scaling factor for the calculation of the look of the sun itself, the look of the earth also uses this value
	const double DIRECT_SUN_INTENSITY;

	//the ratio of incoming light that is reflected by the surface of the earth
	const double EARTH_RED_RATIO;
	const double EARTH_BLUE_RATIO;
	const double EARTH_GREEN_RATIO;

	//scale of exponential scale of density of scatterers
	const double MIE_SCALE_HEIGHT;
	const double RAYLEIGH_SCALE_HEIGHT;

	//Coeffecient of each colour for calculating pixel luminance value used in mapping simulation results to screen colours
	const double RED_MAP_SCALE;
	const double GREEN_MAP_SCALE;
	const double BLUE_MAP_SCALE;

	//physical length of each discrete element of air used when summing scattering effects 
	const double INTEGRAL_STEPS; // 2 or 4/5 or 10?

	// number of threads to spread the pixel rendering over
	const int NUM_THREADS;

	//Linear factor by which to upscale simulation resolution
	const int RES_SCALING;

	//fov in radians
	const double fov;
	//the FOV divided by the width of the display window in pixels
	const double angle_to_pixel_ratio;
	//for viewing on a virtual screen ie non distorted
	double virtual_pixel_width;

	// Render pixels in the array from (zero-based) start_range (inclusive) to end_range (exclusive)
	// Image_Section should point to the element at start range
	static void RenderPixelRange(const int start_range, const int end_range, const CartVec& pos, const SphVec& viewdir, const CartVec& sundir, Pixel* Image_Section, const Simulator* parent);

	//take the view direction corresponding to the centre of the window and return the view vector corresponding to the window coordinates passed (top left being 0,0) 
	CartVec FindPixelDirection(const int, const int, const SphVec&) const;

	// A vector decribing the attenuation of each colour due to the sky between pos and intercept
	CartVec GetSkyAttenuation(const CartVec &pos, const CartVec &Intercept) const;

	//each of the colour finding functions takes only the observer position, the intersection point between the view direction and either the Earth (if looking at the Earth)
	//or the sky (if looking at the sun or sky) and the direction of the sun
	CartVec FindSkyColour(const CartVec &pos, const CartVec &pixeldir, const CartVec &Intercept, const CartVec &sundir) const;
	CartVec FindAtmoColour(const CartVec&, const  CartVec&, const CartVec&) const;
	CartVec FindSunColour(const CartVec&, const  CartVec&, const CartVec&) const;
	CartVec FindEarthColour(const CartVec&, const  CartVec&, const CartVec&) const;
	CartVec FindCloudColour(const Cloud &	theCloud,
							const CartVec &	Pos,
							const CartVec &	PixelDir,
							const CartVec &	Atmo_Intercept,
							const CartVec &	FrontCloud_Intercept,
							const CartVec &	BackCloud_Intercept,
							const CartVec &	SunDir,
							const int		ExtraScatters) const;

	//Map a simulation pixel to a a screen pixel
	Pixel	MapPixel(const Pixel_Unmapped&) const;
	//Upscale image
	void	UpscaleImage(Pixel* Image);
	//Upscale pixel sides
	void	UpscalePixel_Sides(int Screen_x, int Screen_y, Pixel* Image);
	//Upscale pixel centre
	//requires sides to be upscaled before this will work
	void	UpscalePixel_Centre(int Screen_x, int Screen_y, Pixel* Image);
};