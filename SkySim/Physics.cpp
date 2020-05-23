#include "Physics.h"
#include <memory>

PhysicsEngine::PhysicsEngine(Variables loaded_variables) :
	EARTH_RADIUS(loaded_variables.EARTH_RADIUS),
	ATMO_RADIUS(loaded_variables.ATMO_RADIUS),
	MIE_COEFF(loaded_variables.MIE_COEFF),
	MIE_SCALE_HEIGHT(loaded_variables.MIE_SCALE_HEIGHT),
	CLOUD_MIE_COEFF(loaded_variables.CLOUD_MIE_COEFF),
	RAYLEIGH_SCALE_HEIGHT(loaded_variables.RAYLEIGH_SCALE_HEIGHT),
	MIE_MEAN_COSINE(loaded_variables.MIE_MEAN_COSINE),
	MIE_MEAN_COSINE_SQU(MIE_MEAN_COSINE * MIE_MEAN_COSINE),
	RAYLEIGH_BLUE(loaded_variables.RAYLEIGH_BLUE),
	RAYLEIGH_RED(loaded_variables.RAYLEIGH_RED),
	RAYLEIGH_GREEN(loaded_variables.RAYLEIGH_GREEN)
{
}


//Cube is aligned to cooridinate axes.
double PhysicsEngine::ApproxCubeSphereIntVolume(const CartVec& cube_centre, const double side_length, const CartVec& sphere_centre, const double radius, CartVec& out_approx_int_centre) const
{
	out_approx_int_centre.x = 0;
	out_approx_int_centre.y = 0;
	out_approx_int_centre.z = 0;

	const double half_side_length = side_length / 2;
	int closest_x;
	int closest_y;
	int closest_z;
	double closest_dist = DBL_MAX;
	int furthest_x;
	int furthest_y;
	int furthest_z;
	double furthest_dist = 0;
	CubeVertex Cube[8];

	int NumCloser = 0;

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				CubeVertex& Vertex = Cube[i + 2 * j + 4 * k];
				Vertex.pos = cube_centre + CartVec((i - 1) * half_side_length, (j - 1) * half_side_length, (k - 1) * half_side_length);
				Vertex.dist = sqrt((Vertex.pos - sphere_centre).SquMag());
				if (Vertex.dist < closest_dist)
				{
					closest_dist = Vertex.dist;
					closest_x = i;
					closest_y = j;
					closest_z = k;
				}
				if (Vertex.dist > furthest_dist)
				{
					furthest_dist = Vertex.dist;
					furthest_x = i;
					furthest_y = j;
					furthest_z = k;
				}
				if (Vertex.dist <= radius) { NumCloser += 1; Vertex.closer = true; }
			}
		}
	}
	CubeVertex* CloseGroup[4];
	CubeVertex* FarGroup[4];
	CloseGroup[0] = Cube + closest_x + 2 * closest_y + 4 * closest_z;
	FarGroup[0] = Cube + furthest_x + 2 * furthest_y + 4 * furthest_z;
	CloseGroup[1] = Cube + closest_x + ((closest_x == 0) ? 1 : -1) + 2 * closest_y + 4 * closest_z;
	CloseGroup[2] = Cube + closest_x + ((closest_y == 0) ? 1 : -1) * 2 + 2 * closest_y + 4 * closest_z;
	CloseGroup[3] = Cube + closest_x + ((closest_z == 0) ? 1 : -1) * 4 + 2 * closest_y + 4 * closest_z;
	FarGroup[1] = Cube + closest_x + ((closest_x == 0) ? 1 : -1) + 2 * closest_y + 4 * closest_z;
	FarGroup[2] = Cube + closest_x + ((closest_y == 0) ? 1 : -1) * 2 + 2 * closest_y + 4 * closest_z;
	FarGroup[3] = Cube + closest_x + ((closest_z == 0) ? 1 : -1) * 4 + 2 * closest_y + 4 * closest_z;

	double result;
	double lengths[3];
	if (NumCloser == 0)
	{
		out_approx_int_centre = cube_centre;
		return 0;
	}
	else if (NumCloser == 8)
	{
		out_approx_int_centre = cube_centre;
		return side_length * side_length * side_length;
	}
	else if (NumCloser < 4)
	{
		for (int i = 1; i < 4; i++)
		{
			if (CloseGroup[i]->closer)
			{
				lengths[i - 1] = side_length;
			}
			else
			{
				lengths[i - 1] = side_length * ((radius - closest_dist) / (CloseGroup[i]->dist - closest_dist));
			}
		}
		result = lengths[0] * lengths[1] * lengths[2] * 0.5;
	}
	else if (NumCloser > 4)
	{
		for (int i = 1; i < 4; i++)
		{
			if (!FarGroup[i]->closer)
			{
				lengths[i - 1] = side_length;
			}
			else
			{
				lengths[i - 1] = side_length * ((furthest_dist - radius) / (furthest_dist - FarGroup[i]->dist));
			}
		}
		result = side_length * side_length * side_length - lengths[0] * lengths[1] * lengths[2] * 0.5;
	}
	else if (NumCloser == 4)
	{
		double sum_closer_dist = 0;
		double sum_further_dist = 0;
		for (int i = 0; i < 8; i++)
		{
			if (Cube[i].closer)
			{
				sum_closer_dist += Cube[i].dist;
			}
			else
			{
				sum_further_dist += Cube[i].dist;
			}
		}
		result = side_length * side_length * side_length * ((radius * 4 - sum_closer_dist) / (sum_further_dist - sum_closer_dist));
	}

	for (int i = 0; i < 8; i++)
	{
		if (Cube[i].closer)
		{
			out_approx_int_centre = out_approx_int_centre + Cube[i].pos;
		}
	}
	out_approx_int_centre = out_approx_int_centre.Scale(1 / (double)NumCloser);
	return result;
}

CartVec PhysicsEngine::EvaluateRayElementTerm(const CartVec &pos, const CartVec &element, const CartVec &sun_intercept, const double element_size, const double Rayleigh_phase, const double Mie_phase)const
{
	//Return vector stores Red value as x component, green as y and blue as z.

	//height above the surface of the Earth of the scattering element
	double h = sqrt(element.SquMag()) - EARTH_RADIUS;
	//The 'transmission integrals' approximate the integral of the scattering particle density (hence it is different for the different types of scattering) along the line between two points
	//the following two variables are sums of the integrals from both the 'sun-element' line and the 'element-observer' line
	double R_TransInts = (ApproxTransmissionIntegral(pos, element, RAYLEIGH_SCALE_HEIGHT) + ApproxTransmissionIntegral(sun_intercept, element, RAYLEIGH_SCALE_HEIGHT));
	double M_TransInts = (ApproxTransmissionIntegral(pos, element, MIE_SCALE_HEIGHT) + ApproxTransmissionIntegral(sun_intercept, element, MIE_SCALE_HEIGHT));
	//the total factor by which the light scattered from the element is reduced by attenuation due to travelling through the air
	//the particles responsible for Mie scattering may be modelled as also absorbing (rather than scattering away) a small portion of the light,
	//hence a factor of 1.1 scales the Mie scattering coefficient here
	double TotalRedAttenuation = exp(-1 * GetBaseRayleighScatCoeff(Colour::Red) * R_TransInts - 1.1 * GetBaseMieScatCoeff() * M_TransInts);
	double TotalGreenAttenuation = exp(-1 * GetBaseRayleighScatCoeff(Colour::Green) * R_TransInts - 1.1 * GetBaseMieScatCoeff() * M_TransInts);
	double TotalBlueAttenuation = exp(-1 * GetBaseRayleighScatCoeff(Colour::Blue) * R_TransInts - 1.1 * GetBaseMieScatCoeff() * M_TransInts);
	//the scaling of the scattering coefficient for this element relative to that at sea level, caused by the variation in scatterer density with height
	double Rayleigh_Scaling = exp(-1 * h / RAYLEIGH_SCALE_HEIGHT);
	double Mie_Scaling = exp(-1 * h / MIE_SCALE_HEIGHT);
	return CartVec(TotalRedAttenuation * element_size * (GetBaseRayleighScatCoeff(Colour::Red) * Rayleigh_Scaling * Rayleigh_phase + GetBaseMieScatCoeff()* Mie_Scaling * Mie_phase),
		TotalGreenAttenuation * element_size * (GetBaseRayleighScatCoeff(Colour::Green) * Rayleigh_Scaling * Rayleigh_phase + GetBaseMieScatCoeff()* Mie_Scaling * Mie_phase),
		TotalBlueAttenuation * element_size * (GetBaseRayleighScatCoeff(Colour::Blue) * Rayleigh_Scaling * Rayleigh_phase + GetBaseMieScatCoeff()* Mie_Scaling * Mie_phase));
}
double PhysicsEngine::ApproxTransmissionIntegral(const CartVec &x1, const CartVec &x2, const double scale_height) const
{
	//an analytic approximation of the integral of exponentially decreasing scatterer density along a line between two points
	//the approximation states that the height above the earth's surface varies linearly with distance between the two points
	CartVec temp = x1 - x2;
	//distance the between the two points
	double dist = sqrt(temp.SquMag());
	//distance from centre of Earth to point x2
	double intercept = sqrt(x2.SquMag());
	//linear gradient of height between x1 and x2
	double grad = (sqrt(x1.SquMag()) - intercept) / dist;
	//correct this value to only be relative to sea level
	intercept -= EARTH_RADIUS;
	if (grad * grad < DBL_EPSILON)
	{
		//since the final expression includes '1/grad' a different expression, valid in the limit of small gradient, can be used to avoid any issues with dividing by a very small number (or zero)
		return dist * exp(-1 * intercept / scale_height);
	}
	else
	{
		return exp(-1 * intercept / scale_height) * (scale_height / grad) * (1 - exp(-1 * grad * dist / scale_height));
	}
}

//Mathematical functions describing the dependance of scattering amplitude on the angle between incoming and outgoing rays
double PhysicsEngine::RayleighPhaseFunc(const CartVec &ray, const CartVec &sundir) const
{
	double ray_sun_cosine = ray.Dot(sundir);
	return (3 / (16 * M_PI)) * (1 + ray_sun_cosine * ray_sun_cosine);
}
double PhysicsEngine::MiePhaseFunc(const CartVec &ray, const CartVec &sundir) const
{
	double ray_sun_cosine = ray.Dot(sundir);
	double temp = sqrt(1 + MIE_MEAN_COSINE_SQU - 2 * MIE_MEAN_COSINE * ray_sun_cosine);
	return (3 / (8 * M_PI)) * (1 - MIE_MEAN_COSINE_SQU) * (1 + ray_sun_cosine * ray_sun_cosine) / ((2 + MIE_MEAN_COSINE_SQU) * temp * temp * temp);
}
