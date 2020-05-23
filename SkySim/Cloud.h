#pragma once
#include "Vectors.h"
#include "Physics.h"

struct CloudVoxel
{
	CloudVoxel(double width, double denstiy, class Cloud* parent);
	double SideLength;
	double Density;
	double Volume;
	CartVec LocalPos;
	CartVec GlobalPos;
	CartVec EffectivePos;
	class Cloud* Parent;
};

class Cloud
{
public:
	Cloud(PhysicsEngine*);
	~Cloud();

	double Radius;
	double Radius_Squ;
	CartVec CentrePos;

	void Update();

	void SetCentrePos(double X, double Y, double Z);

	double FindExtinction(const CartVec&, const CartVec&, const CartVec&) const;

	double FindCloudEffect(const CartVec&, const CartVec&, const CartVec&, const CartVec&, int ExtraScatters) const;
private:
	const double CLOUD_MIE_COEFF;

	double FindExtinction_FromPoint(const CartVec&, const CartVec&) const;
	double FindExtinction_Internal(const CartVec&, const CartVec&, const double) const;

	//Total Light scattered from given position subject to certain number of scatters within the cloud
	double FindVoxelEffect(const CartVec&, const CartVec&, const CartVec&, int RemainingScatters) const;

	double GetDensity(const CartVec& pos) const;
	CloudVoxel* GetVoxel(const CartVec&) const;

	CartVec FindSurfacePoint(const CartVec&, const CartVec&, double&) const;

	CartVec CornerPos;

	CartVec Velocity;
	CloudVoxel* Voxels;
	int VoxelsWide;
	int VoxelsWide_Squ;
	int TotalVoxels;
	double VoxelHalfWidth;
	double VoxelWidth;
	double VoxelWidth_Cubed;
	PhysicsEngine* Engine;
	double Coeffs[15];
};