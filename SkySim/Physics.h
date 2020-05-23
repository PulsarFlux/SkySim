#pragma once
#include "Vectors.h"

struct Variables
{
	double FOV;
	int SCREEN_WIDTH;
	int SCREEN_HEIGHT;
	double EARTH_RADIUS;
	double ATMO_RADIUS;
	double SUN_INTENSITY;
	double DIRECT_SUN_INTENSITY;
	double EARTH_RED_RATIO;
	double EARTH_BLUE_RATIO;
	double EARTH_GREEN_RATIO;
	double MIE_COEFF;
	double MIE_SCALE_HEIGHT;
	double MIE_MEAN_COSINE;
	double CLOUD_MIE_COEFF;
	double RAYLEIGH_BLUE;
	double RAYLEIGH_RED;
	double RAYLEIGH_GREEN;
	double RAYLEIGH_SCALE_HEIGHT;
	double RED_MAP_SCALE;
	double GREEN_MAP_SCALE;
	double BLUE_MAP_SCALE;
	double INTEGRAL_STEPS;
	int NUM_THREADS;
	int RES_SCALING;
};

enum Colour
{
	Red, Green, Blue
};

class PhysicsEngine
{
public:
	PhysicsEngine(Variables);

	//all distance values are specified in kilometres
	const double EARTH_RADIUS;
	const double ATMO_RADIUS;

	//Mie scattering parameters
	const double MIE_COEFF;
	const double MIE_MEAN_COSINE;
	const double MIE_MEAN_COSINE_SQU;
	const double CLOUD_MIE_COEFF;

	//scale of exponential scale of density of scatterers
	const double MIE_SCALE_HEIGHT;
	const double RAYLEIGH_SCALE_HEIGHT;

	//Rayleigh scattering parameters
	const double RAYLEIGH_BLUE;
	const double RAYLEIGH_RED;
	const double RAYLEIGH_GREEN;

	// Values used in ApproxCubeSphereIntVolume
	struct CubeVertex
	{
		CubeVertex() :
		closer(false),
		pos(),
		dist()
		{}
		bool closer;
		CartVec pos;
		double dist;
	};

	//Cube is aligned to cooridinate axes.
	double ApproxCubeSphereIntVolume(const CartVec& centre_cube, const double side_length, const CartVec& sphere_centre, const double radius, CartVec& approx_int_centre) const;

	CartVec EvaluateRayElementTerm(const CartVec& pos, const CartVec&, const CartVec&, const double, const double, const double) const;
	double ApproxTransmissionIntegral(const CartVec&, const CartVec&, const double) const;

	//Get the sea level scattering coefficients for the passed colour
	inline double GetBaseRayleighScatCoeff(const Colour col) const
	{
		//returns the scattering coefficient at sea level for the colour given by the index argument
		//index = 0, 1, 2 for red, green and blue respectively
		double result;
		switch (col)
		{
		case Colour::Red:
			result = RAYLEIGH_RED;
			break;
		case Colour::Green:
			result = RAYLEIGH_GREEN;
			break;
		case Colour::Blue:
			result = RAYLEIGH_BLUE;
			break;
		}
		return result;
	}
	inline double GetBaseMieScatCoeff() const
	{
		//returns the scattering coefficient at sea level for the colour given by the index argument
		//index = 0, 1, 2 for red, green and blue respectively
		return MIE_COEFF;
		/*switch (col)
		{
		case Colour::Red:
		result = MIE_COEFF;
		break;
		case Colour::Green:
		result = MIE_COEFF;
		break;
		case Colour::Blue:
		result = MIE_COEFF;
		break;
		}
		return result;*/
	}

	//Mathematical functions describing the dependance of scattering amplitude on the angle between incoming and outgoing rays
	double RayleighPhaseFunc(const CartVec&, const CartVec&) const;
	double MiePhaseFunc(const CartVec&, const CartVec&) const;
};
