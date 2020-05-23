#include "Vectors.h"

//For sphere not on origin, PosSph centre of sphere and PosPoint is observation point
bool FindSphereIntersection(const double Radius, const CartVec &PosSph, const CartVec &PosPoint, const CartVec &Dir, CartVec& Intercept, CartVec* Intercept_Far)
{
	bool result;
	result = FindSphereIntersection(Radius, PosPoint - PosSph, Dir, Intercept, Intercept_Far);
	Intercept = Intercept + PosSph;
	if (Intercept_Far)
	{
		*Intercept_Far = *Intercept_Far + PosSph;
	}
	return result;
}
bool FindSphereIntersection(const double Radius, const CartVec &Pos, const CartVec &Dir, CartVec& Intercept,  CartVec* Intercept_Far)
{
	//Analyticly solve quadratic equation produced by looking for intersections between a sphere and a line
	double a = Dir.SquMag();
	double b = 2 * Pos.Dot(Dir);
	double c = Pos.SquMag() - Radius * Radius;
	double determinant = b * b - 4 * a * c;
	//no intersections at all
	if (determinant < 0)
	{
		return false;
	}

	//these values parameterise how far from Pos the intersections are, their sign indicates whether they are forward or backswards with respect to the Dir vector.
	double s_plus = (-b + sqrt(determinant)) / (2 * a);
	double s_minus = (-b - sqrt(determinant)) / (2 * a);
	//If not Intecept_Far only interested in closest positive solution since we assume we are positioned inside the atmosphere
	if (Intercept_Far)
	{
		CartVec result = Pos + Dir.Scale(s_minus);
		Intercept.x = result.x;
		Intercept.y = result.y;
		Intercept.z = result.z;
		result = Pos + Dir.Scale(s_plus);
		Intercept_Far->x = result.x;
		Intercept_Far->y = result.y;
		Intercept_Far->z = result.z;
		if (s_minus <= 0 && s_plus <= 0)
		{
			return false;
		}
	}
	else
	if (s_minus > 0)
	{
		CartVec result = Pos + Dir.Scale(s_minus);
		Intercept.x = result.x;
		Intercept.y = result.y;
		Intercept.z = result.z;
	}
	else if (s_plus > 0)
	{
		CartVec result = Pos + Dir.Scale(s_plus);
		Intercept.x = result.x;
		Intercept.y = result.y;
		Intercept.z = result.z;
	}
	else
	{
		//if both intersections are in the backwards direction
		return false;
	}

	return true;
}
bool FindSphereIntersection(const double Radius, const SphVec &Pos, const SphVec &Dir, SphVec& Intercept, SphVec* Intercept_Far)
{
	//Analyticly solve quadratic equation produced by looking for intersections between a sphere and a line
	double a = Dir.r * Dir.r;
	double b = 2 * Pos.Dot(Dir);
	double c = Pos.r * Pos.r - Radius * Radius;
	double determinant = b * b - 4 * a * c;
	//no intersections at all
	if (determinant < 0)
	{
		return false;
	}

	//these values parameterise how far from Pos the intersections are, their sign indicates whether they are forward or backswards with respect to the Dir vector.
	double s_plus = (-b + sqrt(determinant)) / (2 * a);
	double s_minus = (-b - sqrt(determinant)) / (2 * a);
	//If not Intecept_Far only interested in closest positive solution since we assume we are positioned inside the atmosphere
	if (Intercept_Far)
	{
		SphVec result = Pos + Dir.Scale(s_minus);
		Intercept.r = result.r;
		Intercept.theta = result.theta;
		Intercept.phi = result.phi;
		result = Pos + Dir.Scale(s_plus);
		Intercept_Far->r = result.r;
		Intercept_Far->theta = result.theta;
		Intercept_Far->phi = result.phi;
		if (s_minus <= 0 && s_plus <= 0)
		{
			return false;
		}
	}
	else if (s_minus > 0)
	{
		SphVec result = Pos + Dir.Scale(s_minus);
		Intercept.r = result.r;
		Intercept.theta = result.theta;
		Intercept.phi = result.phi;
	}
	else if (s_plus > 0)
	{
		SphVec result = Pos + Dir.Scale(s_plus);
		Intercept.r = result.r;
		Intercept.theta = result.theta;
		Intercept.phi = result.phi;
	}
	else
	{
		//if both intersections are in the backwards direction
		return false;
	}

	return true;
}