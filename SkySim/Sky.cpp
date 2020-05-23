#include "Sky.h"
#include "Cloud.h"
#include <thread>

Simulator::Simulator(Variables loaded_variables) : 
Engine(loaded_variables),
FOV(loaded_variables.FOV),
SCREEN_WIDTH(loaded_variables.SCREEN_WIDTH),
SCREEN_HEIGHT(loaded_variables.SCREEN_HEIGHT),
EARTH_RADIUS(loaded_variables.EARTH_RADIUS),
ATMO_RADIUS(loaded_variables.ATMO_RADIUS),
SUN_INTENSITY(loaded_variables.SUN_INTENSITY),
DIRECT_SUN_INTENSITY(loaded_variables.DIRECT_SUN_INTENSITY),
EARTH_RED_RATIO(loaded_variables.EARTH_RED_RATIO),
EARTH_BLUE_RATIO(loaded_variables.EARTH_BLUE_RATIO),
EARTH_GREEN_RATIO(loaded_variables.EARTH_GREEN_RATIO),
MIE_SCALE_HEIGHT(loaded_variables.MIE_SCALE_HEIGHT),
RAYLEIGH_SCALE_HEIGHT(loaded_variables.RAYLEIGH_SCALE_HEIGHT),
RED_MAP_SCALE(loaded_variables.RED_MAP_SCALE),
GREEN_MAP_SCALE(loaded_variables.GREEN_MAP_SCALE),
BLUE_MAP_SCALE(loaded_variables.BLUE_MAP_SCALE),
INTEGRAL_STEPS(loaded_variables.INTEGRAL_STEPS),
NUM_THREADS(loaded_variables.NUM_THREADS),
RES_SCALING(loaded_variables.RES_SCALING),
fov(M_PI * FOV / 180),
angle_to_pixel_ratio(fov / SCREEN_WIDTH)
{
	if (FOV < 180)
	{
		virtual_pixel_width = tan(fov / 2) * (2 / (double)SCREEN_WIDTH);
	}
	Clouds = new Cloud*[1];
	Clouds[0] = new Cloud(&Engine);
	(*(Clouds[0])).SetCentrePos(3, 0, 6362);
	Clouds[0]->Update();
}

//given the position of the observer, the direction they are looking in, and the direction of the sun the look of the sky can be computed
//the position of the observer must be within the atmosphere
std::shared_ptr<Pixel> Simulator::ProduceImage(SphVec pos, SphVec viewdir, SphVec sundir)
{
	int total_pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
	//pointer to array of pixels that will be filled in by the simulation computation
	std::unique_ptr<Pixel> Image = std::unique_ptr<Pixel>(new Pixel[total_pixels * RES_SCALING * RES_SCALING]);

	CartVec SunDir = sundir.Cartesian();
	CartVec Pos = pos.Cartesian();

	//make sure direction vectors are normalised
	SunDir.Normalise();

	std::thread** threads = new std::thread*[NUM_THREADS];
	int total_pixels_assigned = 0;
	int base_pixels_per_thread = total_pixels / NUM_THREADS;
	int extra_pixels = total_pixels % NUM_THREADS;
	// assign all pixels to be rendered on a threaded process
	for (int i = 0; i < NUM_THREADS; i++)
	{
		int end_pixel_range;
		if (total_pixels - total_pixels_assigned <= base_pixels_per_thread)
		{
			end_pixel_range = total_pixels;
		}
		else if (i < extra_pixels)
		{
			end_pixel_range = total_pixels_assigned + base_pixels_per_thread + 1;
		}
		else
		{
			end_pixel_range = total_pixels_assigned + base_pixels_per_thread;
		}
		threads[i] = new std::thread(&RenderPixelRange, total_pixels_assigned, end_pixel_range, Pos, viewdir, SunDir, Image.get() + total_pixels_assigned, this);
		total_pixels_assigned = end_pixel_range;
	}
	//Wait for every thread to finish
	for (int i = 0; i < NUM_THREADS; i++)
	{
		(*threads[i]).join();
	}
	for (int i = 0; i < NUM_THREADS; i++)
	{
		delete threads[i];
	}
	delete[] threads;

	UpscaleImage(Image.get());

	return std::shared_ptr<Pixel>(std::move(Image));
}

void Simulator::RenderPixelRange(const int start_range, const int end_range, const CartVec &Pos, const SphVec &viewdir, const CartVec &SunDir, Pixel* Image_Section, const Simulator* parent)
{
	//Vector that will be used to hold line/sphere intersection points
	CartVec Intercept = CartVec();
	Pixel_Unmapped unmapped_result;

	for (int pixel_counter = start_range; pixel_counter < end_range; pixel_counter++)
	{
		int k = pixel_counter / parent->SCREEN_WIDTH;
		int i = pixel_counter % parent->SCREEN_WIDTH;

		//find the vector direction from the observation point of the current pixel being computed
		CartVec PixelDir = parent->FindPixelDirection(i, k, viewdir);

		//make sure direction vectors are normalised
		PixelDir.Normalise();

		//check if the current pixel direction is looking at the Earth, if so 'Intercept' is set to the point that the pixel is looking at
		if (FindSphereIntersection(parent->EARTH_RADIUS, Pos, PixelDir, Intercept))
		{
			CartVec Result = parent->FindEarthColour(Pos, Intercept, SunDir);
			unmapped_result.Red = Result.x;
			unmapped_result.Green = Result.y;
			unmapped_result.Blue = Result.z;
		}
		//It should not be possible to fail this second 'if' as long as the observer is inside in the atmosphere
		//the 'Intercept' variable will be set to the point where the the line in the pixel direction leaves the atmosphere
		else if (FindSphereIntersection(parent->ATMO_RADIUS, Pos, PixelDir, Intercept))
		{
			CartVec Front_Intercept = CartVec();
			CartVec Back_Intercept = CartVec();
			bool Cloud = false;

			/* Enable/Disbale clouds  */ if (true)
			{
				CartVec CloudRelPos(parent->Clouds[0]->CentrePos - Pos);
				double CloudDist = sqrt(CloudRelPos.SquMag());
				CloudRelPos.Normalise();
				if (CloudRelPos.Dot(PixelDir) > cos(parent->Clouds[0]->Radius / CloudDist))
				{
					if (FindSphereIntersection(parent->Clouds[0]->Radius, parent->Clouds[0]->CentrePos, Pos, PixelDir, Front_Intercept, &Back_Intercept))
					{
						Cloud = true;
						CartVec Result = parent->FindCloudColour(*(parent->Clouds[0]), Pos, PixelDir, Intercept, Front_Intercept, Back_Intercept, SunDir, 1);
						unmapped_result.Red = Result.x;
						unmapped_result.Green = Result.y;
						unmapped_result.Blue = Result.z;
					}
				}
			}
			if (!Cloud)
			{
				CartVec Result = parent->FindSkyColour(Pos, PixelDir, Intercept, SunDir);
				unmapped_result.Red = Result.x;
				unmapped_result.Green = Result.y;
				unmapped_result.Blue = Result.z;
			}
		}
		else
		{
			//Space?
			unmapped_result.Red = 0;
			unmapped_result.Green = 0;
			unmapped_result.Blue = 0;
		}
		// Map simulation pixel to screen pixel
		Pixel Result = parent->MapPixel(unmapped_result);
		Image_Section[i * parent->RES_SCALING + k * parent->RES_SCALING * parent->RES_SCALING * parent->SCREEN_WIDTH - start_range].Red = Result.Red;
		Image_Section[i * parent->RES_SCALING + k * parent->RES_SCALING * parent->RES_SCALING * parent->SCREEN_WIDTH - start_range].Green = Result.Green;
		Image_Section[i * parent->RES_SCALING + k * parent->RES_SCALING * parent->RES_SCALING * parent->SCREEN_WIDTH - start_range].Blue = Result.Blue;
	}
}

//take the view direction corresponding to the centre of the window and return the view vector corresponding to the window coordinates passed (top left being 0,0) 
CartVec Simulator::FindPixelDirection(const int pixel_x, const int pixel_y, const SphVec &viewdir) const
{
	//displacement from centre of screen
	double x_offset = -1 * (SCREEN_WIDTH / 2.0) + pixel_x;
	double y_offset = -1 * (SCREEN_HEIGHT / 2.0) + pixel_y;
	if (FOV >= 180)
	{
		SphVec pixeldir = viewdir;
		//the direction in the centre of the window has its spherical coordinates incremented in proportion to the x and y offsets (phi with x and theta with y)
		//this is not a standard screen space projection and produces large distortions near the poles but is simpler and can cope with any value for the field of view
		pixeldir.Rotate(y_offset * angle_to_pixel_ratio, x_offset * angle_to_pixel_ratio);
		return pixeldir.Cartesian();
	}
	else
	{
		double cos_p = cos(viewdir.phi);
		double sin_p = sin(viewdir.phi);
		double cos_t = cos(viewdir.theta);
		double sin_t = sin(viewdir.theta);
		CartVec unit_theta(cos_t * cos_p, cos_t * sin_p, -1 * sin_t); //this is the screen y unit vector
		CartVec unit_minus_phi(sin_p, -1 * cos_p, 0); //this is the screen x unit vector
		CartVec pixeldir(viewdir.Cartesian());
		pixeldir.Normalise();
		pixeldir = pixeldir + unit_theta.Scale(y_offset * virtual_pixel_width) + unit_minus_phi.Scale(x_offset * virtual_pixel_width);
		pixeldir.Normalise();
		return pixeldir;
	}
}

CartVec Simulator::GetSkyAttenuation(const CartVec &pos, const CartVec &Intercept) const
{
	//for explanation of most of the operations performed in this function see 'EvaluateRayElementTerm'

	//this function simply models the attenuation of the sun's light by scattering of the light into different directions
	double R_TransInt = Engine.ApproxTransmissionIntegral(pos, Intercept, RAYLEIGH_SCALE_HEIGHT);
	double M_TransInt = Engine.ApproxTransmissionIntegral(pos, Intercept, MIE_SCALE_HEIGHT);
	double TotalRedAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Red) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
	double TotalGreenAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Green) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
	double TotalBlueAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Blue) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
	return CartVec(TotalRedAttenuation, TotalGreenAttenuation, TotalBlueAttenuation);
}

//each of the colour finding functions takes only the observer position, the intersection point between the view direction and either the Earth (if looking at the Earth)
//or the sky (if looking at the sun or sky) and the direction of the sun
CartVec Simulator::FindAtmoColour(const CartVec &pos, const CartVec &Intercept, const CartVec &sundir) const
	{
		//We shall consider only the effect of single scattering

		//find the total length of sky being looked through and its direction from the observer
		CartVec Ray = Intercept - pos;
		double dist = sqrt(Ray.SquMag());
		Ray.Normalise();

		//the value that will be used to keep track of the integral across the sky
		double dist_remaining = dist;

		CartVec Total = CartVec(); // store total colour information

		//two variables that will hold a different vector for each discrete integral element
		CartVec Element = CartVec();
		CartVec ElementSun_intercept = CartVec();

		//the scattering phase functions depend only on the the angle between the original light direction and the outwards direction of the scattered light
		//since this value is fixed for all points in the sky lying along one view direction it may be evaluated here
		double Rayleigh_Phase_Value = Engine.RayleighPhaseFunc(Ray, sundir);
		double Mie_Phase_Value = Engine.MiePhaseFunc(Ray, sundir);

		double integral_step = dist / INTEGRAL_STEPS;
		if (integral_step < 5){ integral_step = 5; }

		//sum scattered light from each 'element' of air that lies along the view direction
		while (dist_remaining > 0)
		{
			//each element length is either the constant integral step size or the length of air left, if it is less
			double step;
			if (dist_remaining > integral_step)
			{
				step = integral_step;
			}
			else
			{
				step = dist_remaining;
			}
			//find the positional vector of the element's centre with respect to the Earth
			Element = Ray.Scale((dist - dist_remaining) + step / 2) + pos;
			//if the element itself cannot see the sun then it makes no contribution to the scattered light
			if (!FindSphereIntersection(EARTH_RADIUS, Element, sundir, ElementSun_intercept))
			{
				//otherwise 'ElementSun_intercept' is set to the intersection between the atmospheres edge and the line from the element towards the sun
				FindSphereIntersection(ATMO_RADIUS, Element, sundir, ElementSun_intercept);
				//calculate the light scattered from this element and add it to the total light that is scattered along this direction
				Total = Total + Engine.EvaluateRayElementTerm(pos, Element, ElementSun_intercept, step, Rayleigh_Phase_Value, Mie_Phase_Value);
			}
			dist_remaining -= step;
		}
		//scale the light by a global const chosen to get the light into a reasonable value for mapping and display
		Total = Total.Scale(SUN_INTENSITY);

		return Total;
	}

CartVec Simulator::FindSkyColour(const CartVec &pos, const CartVec &pixeldir, const CartVec &Intercept, const CartVec &sundir) const
{
	CartVec Result;
	//Check if the view direction is within the angular radius of the sun, ie if it is looking directly at the sun
	if (sundir.Dot(pixeldir) > cos(SUN_ANGULAR_RADIUS))
	{
		Result = FindSunColour(pos, Intercept, sundir);
	}
	else
	{
		//If not looking at the sun then must be looking into the open sky
		Result = FindAtmoColour(pos, Intercept, sundir);
	}
	return Result;
}

CartVec Simulator::FindSunColour(const CartVec &pos, const CartVec &Intercept, const CartVec &sundir) const
{
	CartVec SkyAttenuation(GetSkyAttenuation(pos, Intercept));
	return SkyAttenuation.Scale(DIRECT_SUN_INTENSITY);
}


CartVec Simulator::FindEarthColour(const CartVec &pos, const CartVec &Intercept, const CartVec &sundir) const
	{
		//for explanation of most of the operations performed in this function see 'EvaluateRayElementTerm'

		CartVec result = CartVec();
		CartVec Earth_SunIntercept = CartVec();
		//if the point on the Earth cannot 'see' the sun then a zero (black) colour vector is returned
		if (!FindSphereIntersection(EARTH_RADIUS - 1, Intercept, sundir, Earth_SunIntercept))
		{
			FindSphereIntersection(ATMO_RADIUS, Intercept, sundir, Earth_SunIntercept);
			//the earth is modelled (though this is far from the case) as reflecting diffusely only the light directly from the sun
			CartVec Sun = FindSunColour(Intercept, Earth_SunIntercept, sundir);
			//the final colour also includes the effect of the sky in between the point on the Earth and the observer
			CartVec Sky = FindAtmoColour(pos, Intercept, sundir);
			double R_TransInt = Engine.ApproxTransmissionIntegral(pos, Intercept, RAYLEIGH_SCALE_HEIGHT);
			double M_TransInt = Engine.ApproxTransmissionIntegral(pos, Intercept, MIE_SCALE_HEIGHT);
			double TotalRedAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Red) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
			double TotalGreenAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Green) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
			double TotalBlueAttenuation = exp(-1 * Engine.GetBaseRayleighScatCoeff(Colour::Blue) * R_TransInt - 1.1 * Engine.GetBaseMieScatCoeff() * M_TransInt);
			result.x = TotalRedAttenuation * Sun.x * EARTH_RED_RATIO + Sky.x;
			result.y = TotalBlueAttenuation * Sun.y * EARTH_GREEN_RATIO + Sky.y;
			result.z = TotalGreenAttenuation * Sun.z * EARTH_BLUE_RATIO + Sky.z;
		}

		return result;
	}


CartVec Simulator::FindCloudColour(	const Cloud &theCloud,
									const CartVec &Pos, 
									const CartVec &PixelDir, 
									const CartVec &Atmo_Intercept, 
									const CartVec &FrontCloud_Intercept, 
									const CartVec &BackCloud_Intercept,
									const CartVec &SunDir, 
									const int ExtraScatters) const
{
	
	CartVec FinalResult = CartVec();
	CartVec Sun = CartVec();
	CartVec Intercept = CartVec();

	CartVec Sky
	(
		FindSkyColour(Pos, PixelDir, Atmo_Intercept, SunDir)
	);
	double CloudAttenuation = theCloud.FindExtinction(PixelDir, FrontCloud_Intercept, BackCloud_Intercept);

	//check if the cloud-sun line is looking at the Earth, if so 'Intercept' is set to the point that it is looking at
	if (FindSphereIntersection(EARTH_RADIUS, FrontCloud_Intercept, SunDir, Intercept))
	{
	}
	//It should not be possible to fail this second 'if' as long as the observer is inside in the atmosphere
	//the 'Intercept' variable will be set to the point where the the line in the sun direction leaves the atmosphere
	else if (FindSphereIntersection(ATMO_RADIUS, FrontCloud_Intercept, SunDir, Intercept))
	{
		// Sun not blocked by Earth so use Sun as the cloud's light source.
		Sun = FindSunColour(Pos, Intercept, SunDir);
	}

	//Fraction of recieved light intensity scatted from cloud towards viewing point
	double CloudEffect = theCloud.FindCloudEffect(PixelDir, FrontCloud_Intercept, BackCloud_Intercept, SunDir, ExtraScatters);

	//Effect of the sky between the cloud and the viewing point
	CartVec SkyAttenuation(GetSkyAttenuation(Pos, FrontCloud_Intercept));

	FinalResult.x = SkyAttenuation.x * Sun.x * CloudEffect + Sky.x  * CloudAttenuation;
	FinalResult.y = SkyAttenuation.y * Sun.y * CloudEffect + Sky.y  * CloudAttenuation;
	FinalResult.z = SkyAttenuation.z * Sun.z * CloudEffect + Sky.z  * CloudAttenuation;

	return FinalResult;
}

//Map a simulation pixel to a a screen pixel
Pixel Simulator::MapPixel(const Pixel_Unmapped &pix) const
{
	Pixel result;
	//Total percieved luminance of the colours
	double Luminance = RED_MAP_SCALE * pix.Red + GREEN_MAP_SCALE * pix.Green + BLUE_MAP_SCALE * pix.Blue;
	//The value for the highest RGB component of the screen pixel
	double Scale = 255 * Luminance / (Luminance + 1);
	//Find which component is the highest and set that to the above scale, whilst keeping the other colours in proportion
	if (pix.Red >= pix.Green && pix.Red >= pix.Blue)
	{
		result.Red = Scale;
		result.Green = (Scale / pix.Red) * pix.Green;
		result.Blue = (Scale / pix.Red) * pix.Blue;
	}
	else if (pix.Green >= pix.Red && pix.Green >= pix.Blue)
	{
		result.Green = Scale;
		result.Red = (Scale / pix.Green) * pix.Red;
		result.Blue = (Scale / pix.Green) * pix.Blue;
	}
	else if (pix.Blue >= pix.Red && pix.Blue >= pix.Green)
	{
		result.Blue = Scale;
		result.Red = (Scale / pix.Blue) * pix.Red;
		result.Green = (Scale / pix.Blue) * pix.Green;
	}
	else
	{
		result.Blue = 255;
		result.Red = 255;
		result.Green = 255;
	}
	return result;
}

//Upscale image
void Simulator::UpscaleImage(Pixel* Image)
{
	for (int i = 0; i < SCREEN_WIDTH; i++)
	{
		for (int k = 0; k < SCREEN_HEIGHT; k++)
		{
			UpscalePixel_Sides(i, k, Image);
		}
	}
	for (int i = 0; i < SCREEN_WIDTH; i++)
	{
		for (int k = 0; k < SCREEN_HEIGHT; k++)
		{
			UpscalePixel_Centre(i, k, Image);
		}
	}
}

//Upscale pixel sides
void Simulator::UpscalePixel_Sides(int Screen_x, int Screen_y, Pixel* Image)
{	
	const int Line_Width = RES_SCALING * SCREEN_WIDTH;

	//Number of pixels along in render space due to x coordinate
	const int render_x = Screen_x * RES_SCALING;
	//Number of pixels along in render space due to y coordinate
	const int render_y = Screen_y * Line_Width * RES_SCALING;

	const int x_sample_dir = Screen_x == SCREEN_WIDTH - 1 ? -1 : 1;
	const int y_sample_dir = Screen_y == SCREEN_HEIGHT - 1 ? -1 : 1;

	const int base = render_x + render_y;

	Pixel_Unmapped x_diff; // Unmapped so that values less than zero do not under flow
	int const x_offset = base + RES_SCALING * x_sample_dir;
	x_diff.Red = (Image[x_offset].Red - Image[base].Red) * x_sample_dir;
	x_diff.Blue = (Image[x_offset].Blue - Image[base].Blue) * x_sample_dir;
	x_diff.Green = (Image[x_offset].Green - Image[base].Green) * x_sample_dir;
	Pixel_Unmapped y_diff; // Unmapped so that values less than zero do not under flow
	int const y_offset = base + Line_Width * RES_SCALING * y_sample_dir;
	y_diff.Red = (Image[y_offset].Red - Image[base].Red) * y_sample_dir;
	y_diff.Blue = (Image[y_offset].Blue - Image[base].Blue) * y_sample_dir;
	y_diff.Green = (Image[y_offset].Green - Image[base].Green) * y_sample_dir;

	for (int i = 1; i < RES_SCALING; i++)
	{
		// X

		int temp = Image[base].Red + (i / (double)RES_SCALING) * x_diff.Red;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i].Red = temp;

		temp = Image[base].Blue + (i / (double)RES_SCALING) * x_diff.Blue;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i].Blue = temp;

		temp = Image[base].Green + (i / (double)RES_SCALING) * x_diff.Green;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i].Green = temp;

		// Y

		temp = Image[base].Red + (i / (double)RES_SCALING) * y_diff.Red;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i * Line_Width].Red = temp;

		temp = Image[base].Blue + (i / (double)RES_SCALING) * y_diff.Blue;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i * Line_Width].Blue = temp;

		temp = Image[base].Green + (i / (double)RES_SCALING) * y_diff.Green;
		//Keep in range 0-255
		temp = temp > 255 ? 255 : temp;
		temp = temp < 0 ? 0 : temp;
		Image[base + i * Line_Width].Green = temp;
	}
}

void Simulator::UpscalePixel_Centre(int Screen_x, int Screen_y, Pixel* Image)
{
	const int Line_Width = RES_SCALING * SCREEN_WIDTH;

	//Number of pixels along in render space due to x coordinate
	const int render_x = Screen_x * RES_SCALING;
	//Number of pixels along in render space due to y coordinate
	const int render_y = Screen_y * Line_Width * RES_SCALING;

	const int x_sample_dir = Screen_x == SCREEN_WIDTH - 1 ? -1 : 1;
	const int y_sample_dir = Screen_y == SCREEN_HEIGHT - 1 ? -1 : 1;

	const int base = render_x + render_y;

	for (int x = 1; x < RES_SCALING; x++)
	{
		for (int y = 1; y < RES_SCALING; y++)
		{
			Pixel_Unmapped x_diff; // Unmapped so that values less than zero do not under flow
			int const x_base = base + y * Line_Width;
			int const x_offset = x_base + RES_SCALING * x_sample_dir;
			x_diff.Red = (Image[x_offset].Red - Image[x_base].Red) * x_sample_dir;
			x_diff.Blue = (Image[x_offset].Blue - Image[x_base].Blue) * x_sample_dir;
			x_diff.Green = (Image[x_offset].Green - Image[x_base].Green) * x_sample_dir;

			Pixel_Unmapped y_diff; // Unmapped so that values less than zero do not under flow
			int const y_base = base + x;
			int const y_offset = y_base + Line_Width * RES_SCALING * y_sample_dir;
			y_diff.Red = (Image[y_offset].Red - Image[y_base].Red) * y_sample_dir;
			y_diff.Blue = (Image[y_offset].Blue - Image[y_base].Blue) * y_sample_dir;
			y_diff.Green = (Image[y_offset].Green - Image[y_base].Green) * y_sample_dir;

			int temp = 0.5 * (Image[x_base].Red + (x / (double)RES_SCALING) * x_diff.Red + Image[y_base].Red + (y / (double)RES_SCALING) * y_diff.Red);
			//Keep in range 0-255
			temp = temp > 255 ? 255 : temp;
			temp = temp < 0 ? 0 : temp;
			Image[base + x + y * Line_Width].Red = temp;

			temp = 0.5 * (Image[x_base].Blue + (x / (double)RES_SCALING) * x_diff.Blue + Image[y_base].Blue + (y / (double)RES_SCALING) * y_diff.Blue);
			//Keep in range 0-255
			temp = temp > 255 ? 255 : temp;
			temp = temp < 0 ? 0 : temp;
			Image[base + x + y * Line_Width].Blue = temp;

			temp = 0.5 * (Image[x_base].Green + (x / (double)RES_SCALING) * x_diff.Green + Image[y_base].Green + (y / (double)RES_SCALING) * y_diff.Green);
			//Keep in range 0-255
			temp = temp > 255 ? 255 : temp;
			temp = temp < 0 ? 0 : temp;
			Image[base + x + y * Line_Width].Green = temp;
		}
	}
}