#pragma once
#define _USE_MATH_DEFINES
#include <math.h>

//template <class T>
//A vector using Cartesian coordinates
class CartVec
{
public:
	CartVec()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	CartVec(double X, double Y, double Z)
	{
		x = X;
		y = Y;
		z = Z;
	}
	double x;
	double y;
	double z;
	//Vector dot product
	inline double Dot(const CartVec &vec) const
	{
		return (x * vec.x + y * vec.y + z * vec.z);
	}
	//Squared magnitude
	inline double SquMag() const
	{
		return (x * x + y * y + z * z);
	}
	inline CartVec Scale(double factor) const
	{
		return CartVec(x * factor, y * factor, z * factor);
	}
	//Change vector such that it points in the same direction but has magnitude one
	inline void Normalise()
	{
		double mag = sqrt(SquMag());
		if (mag != 0)
		{
			x = x / mag;
			y = y / mag;
			z = z / mag;
		}
	}
	inline CartVec operator+(const CartVec& vec) const
	{
		return CartVec(x + vec.x, y + vec.y, z + vec.z);
	}
	inline CartVec operator-(const CartVec& vec) const
	{
		return CartVec(x - vec.x, y - vec.y, z - vec.z);
	}
};

//template <class T>
//A vector using spherical polar coordinates
class SphVec
{
public:
	double r;
	double theta;
	double phi;
	SphVec(double R, double Theta, double Phi)
	{
		r = R;
		theta = Theta;
		phi = Phi;
	}
	//Vector dot product
	inline double Dot(const SphVec &vec) const
	{
		return ((r*vec.r)*(sin(theta)*sin(vec.theta)*cos(phi - vec.phi) + cos(theta)*cos(vec.theta)));
	}
	inline SphVec Scale(double factor) const
	{
		return SphVec(r * factor, theta, phi);
	}
	//Not really a true rotation, just shifting each angular coordinate by the specified amount
	void Rotate(double theta_amount, double phi_amount)
	{
		//Must do additional operations to make sure the angles stay within the range expected for each coordinate
		phi += phi_amount;
		if (phi > 2 * M_PI) phi -= 2 * M_PI;
		if (phi < 0) phi += 2 * M_PI;
		theta += theta_amount;
		if (theta > M_PI)
		{
			theta = 2 * M_PI - theta;
			phi += M_PI;
			if (phi > 2 * M_PI) phi -= 2 * M_PI;
		}
		if (theta < 0)
		{
			theta = -1 * theta;
			phi += M_PI;
			if (phi > 2 * M_PI) phi -= 2 * M_PI;
		}
	}
	//Change vector such that it points in the same direction but has magnitude one
	inline void Normalise()
	{
		r = 1;
	}
	//return the cartesian representation of the same vector
	inline CartVec Cartesian() const
	{
		return CartVec(r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta));
	}
	SphVec operator+(const SphVec& vec) const
	{
		double Temp;
		double R = sqrt(r*r + vec.r * vec.r + 2 * Dot(vec));
		double c1 = cos(theta);
		double c2 = cos(vec.theta);
		Temp = (r*c1 + vec.r*c2) / R;
		double Theta;
		//The inverse cos function cannot take arguments outside the range -1 to 1, however due to doubleing point operations values can end up outside this range
		//these cases must be dealt with manually be assuming any value greater than 1 is supposed to 1 and similarly for -1
		if (Temp > 1)
		{
			Theta = 0;
		}
		else if (Temp < -1)
		{
			Theta = M_PI;
		}
		else
		{
			Theta = acos(Temp);
		}
		Temp = (r*sin(theta)*cos(phi) + vec.r*sin(vec.theta)*cos(vec.phi)) / sqrt(R*R - r*r*c1*c1 - vec.r*vec.r*c2*c2 - 2 * r*vec.r*c1*c2);
		double Phi;
		if (Temp > 1)
		{
			Phi = 0;
		}
		else if (Temp < -1)
		{
			Phi = M_PI;
		}
		else
		{
			Phi = acos(Temp);
		}
		return SphVec(R, Theta, Phi);
	}
	SphVec operator-(const SphVec& vec) const
	{
		//rotate the vector such that it becomes minus the original
		double finalphi = vec.phi + M_PI;
		if (finalphi > 2 * M_PI) finalphi -= 2 * M_PI;
		SphVec minusvec = SphVec(vec.r, M_PI - vec.theta, finalphi);
		//the minus vector can then be added as above to perform an overall subtraction
		double Temp;
		double R = sqrt(r*r + minusvec.r * minusvec.r + 2 * Dot(minusvec));
		double c1 = cos(theta);
		double c2 = cos(minusvec.theta);
		Temp = (r*c1 + minusvec.r*c2) / R;
		double Theta;
		if (Temp > 1)
		{
			Theta = 0;
		}
		else if (Temp < -1)
		{
			Theta = M_PI;
		}
		else
		{
			Theta = acos(Temp);
		}
		Temp = (r*sin(theta)*cos(phi) + minusvec.r*sin(minusvec.theta)*cos(minusvec.phi)) / sqrt(R*R - r*r*c1*c1 - minusvec.r*minusvec.r*c2*c2 - 2 * r*minusvec.r*c1*c2);
		double Phi;
		if (Temp > 1)
		{
			Phi = 0;
		}
		else if (Temp < -1)
		{
			Phi = M_PI;
		}
		else
		{
			Phi = acos(Temp);
		}
		return SphVec(R, Theta, Phi);
	}
};

//Analytical solvers of intersections between spheres and a directed line, all specified by arguments, the relevant intercept point for the simulation is put into the Intercept 'by reference' argument
//Returns true if there is an intersection in the forward direction of the line
//Radius is the radius of the sphere, assumed centred on the origin of the coordinate system used by the Pos vector
//Pos is the position of the observer and hence the point past which 'forward' is defined
//Dir is the direction of the line
bool FindSphereIntersection(const double, const CartVec&, const CartVec&, CartVec&, CartVec* = nullptr);
bool FindSphereIntersection(const double, const SphVec&, const SphVec&, SphVec&, SphVec* = nullptr);
//For sphere not on origin
bool FindSphereIntersection(const double, const CartVec&, const CartVec&, const CartVec &, CartVec& , CartVec* = nullptr);