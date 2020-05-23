#include "Cloud.h"
#include <memory>
#include <random>

CloudVoxel::CloudVoxel(double width, double denstiy, class Cloud* parent) :
	SideLength(width),
	Density(denstiy),
	LocalPos(),
	GlobalPos(),
	Parent(parent)
{
}

Cloud::Cloud(PhysicsEngine* engine) :
	CLOUD_MIE_COEFF(engine->CLOUD_MIE_COEFF),
	Radius(0.5),
	Radius_Squ(Radius * Radius),
	CornerPos(),
	CentrePos(Radius, Radius, Radius),
	Velocity(),
	VoxelsWide(10),
	VoxelsWide_Squ(VoxelsWide * VoxelsWide),
	TotalVoxels(VoxelsWide * VoxelsWide * VoxelsWide),
	VoxelHalfWidth(Radius / (double)VoxelsWide),
	VoxelWidth(2 * VoxelHalfWidth),
	VoxelWidth_Cubed(VoxelWidth * VoxelWidth * VoxelWidth),
	Engine(engine)
{	

	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0, 1);


	Voxels = (CloudVoxel*)malloc(sizeof(CloudVoxel) * TotalVoxels);	

	for (int i = 0; i < 15; i++)
	{
		Coeffs[i] = dis(gen);
	}

	/*
	for (int i = 0; i < TotalVoxels; i++)
	{
		if ((Voxels[i].GlobalPos - CentrePos).SquMag() > (Radius + 1.8 * VoxelWidth) * (Radius + 1.8 * VoxelWidth))
		{
			Voxels[i].Density = 0.0f;
		}
		else
		{
			Voxels[i].Density = 1.0f;
		}
	}*/
}

Cloud::~Cloud()
{
	free(Voxels);
}

void Cloud::Update()
{
	for (int i = 0; i < VoxelsWide; i++)
	{
		for (int j = 0; j < VoxelsWide; j++)
		{
			for (int k = 0; k < VoxelsWide; k++)
			{
				CloudVoxel* Voxel = Voxels + i + j * VoxelsWide + k * VoxelsWide_Squ;
				Voxel->GlobalPos = CornerPos + CartVec(i * VoxelWidth, j * VoxelWidth, k * VoxelWidth);
				Voxel->Volume = Engine->ApproxCubeSphereIntVolume(Voxel->GlobalPos, VoxelWidth, CentrePos, Radius, Voxel->EffectivePos);
			}
		}
	}
}

double Cloud::FindCloudEffect(const CartVec& viewdir, const CartVec& front_intercept, const CartVec& back_intercept, const CartVec& sundir, int ExtraScatters) const
{
	CartVec Pos(front_intercept);
	double distleft( sqrt((front_intercept - back_intercept).SquMag()) );
	double maxStep(VoxelHalfWidth);
	double total(0);
	while (distleft > 0)
	{
		double step(maxStep);
		if (distleft < maxStep)
		{
			step = distleft;
		}
		Pos = Pos + viewdir.Scale(step);

		if (GetDensity(Pos) > DBL_EPSILON)
		{
			total += step * FindVoxelEffect(Pos, viewdir, sundir, ExtraScatters);
		}

		distleft -= step;
	}
	return total;
}

double Cloud::FindExtinction(const CartVec& viewdir, const CartVec& front_intercept, const CartVec& back_intercept) const
{
	double dist = sqrt((front_intercept - back_intercept).SquMag());
	return FindExtinction_Internal(viewdir, front_intercept, dist);
}

void Cloud::SetCentrePos(double X, double Y, double Z)
{
	CentrePos.x = X;
	CentrePos.y = Y;
	CentrePos.z = Z;
	CornerPos.x = CentrePos.x - Radius;
	CornerPos.y = CentrePos.y - Radius;
	CornerPos.z = CentrePos.z - Radius;
}

//Extinction along the line from pos and looking along viewdir
double Cloud::FindExtinction_FromPoint(const CartVec& pos, const CartVec& viewdir) const
{
	double dist;
	FindSurfacePoint(pos, viewdir, dist);
	return FindExtinction_Internal(viewdir, pos, dist);
}

double Cloud::FindExtinction_Internal(const CartVec& viewdir, const CartVec& from, const double dist) const
{
	CartVec Pos(from);
	double maxStep(VoxelHalfWidth);
	double total = 0; // dist; //TODO - we are assuimg cloud density of 1 for now.
	double distleft = dist;
	while (distleft > 0)
	{
		double step(maxStep);
		if (distleft < maxStep)
		{
			step = distleft;
		}
		Pos = Pos + viewdir.Scale(step);

		if (GetDensity(Pos) > DBL_EPSILON)
		{
			total += step * GetDensity(Pos);
		}

		distleft -= step;
	}
	return exp(-1 * CLOUD_MIE_COEFF * total);
}



//Looking at voxelpos along viewdir
double Cloud::FindVoxelEffect(const CartVec& voxelpos, const CartVec& viewdir, const CartVec& sundir, int RemainingScatters) const
{
	double result = 0;
	CloudVoxel* voxel = GetVoxel(voxelpos);
	double Extinc_In = FindExtinction_FromPoint(voxelpos, viewdir.Scale(-1.0));
	//if (Extinc_In < 0.001) { return 0; }
	double Extinc_Out = FindExtinction_FromPoint(voxelpos, sundir);
	//if (Extinc_Out < 0.001) { return 0; }
	result += Engine->MiePhaseFunc(viewdir, sundir) * Extinc_In * Extinc_Out;
	if (RemainingScatters != 0)
	{ 
		double secondary_contrib = 0;
		for (int i = 0; i < TotalVoxels; i++)
		{
			if (Voxels + i != voxel)
			{
				CartVec otherVoxelPos = Voxels[i].EffectivePos;
				if (GetDensity(otherVoxelPos) > 0.01 && Voxels[i].Volume > DBL_EPSILON)
				{	//Integrate over the cloud: fraction scattered and deposited per unit dist (GetVoxeleffect) * Fraction scattered through angle at original voxel (Mie Phase Func) 
					// * Volume element (volume of voxel) / Area scattered to (4 PI * dist ^ 2 )
					CartVec RelPos = otherVoxelPos - voxelpos;
					double squ_dist = RelPos.SquMag();
					RelPos.Normalise();
					secondary_contrib += Voxels[i].Volume * (1 / squ_dist) * Engine->MiePhaseFunc(RelPos, viewdir) * FindVoxelEffect(otherVoxelPos, RelPos, sundir, RemainingScatters - 1);
				}
			}
		}
		result += (1 / (4 * M_PI)) * secondary_contrib;

	}
	//outside this function will need to multiply this value by the step length taken
	return result * CLOUD_MIE_COEFF * GetDensity(voxelpos);
}

double Cloud::GetDensity(const CartVec& pos) const
{
	static float freqs[5] = { M_PI / (Radius * 2), 2 * M_PI / (Radius * 2), 3 * M_PI / (Radius * 2), 4 * M_PI / (Radius * 2), 5 * M_PI / (Radius * 2) };

	int i = 0;
	double total = 0;
	double value = 0;
	const CartVec relPos = (pos - CornerPos);

	for (; i < 5; i++)
	{
		value += Coeffs[i] * sin(freqs[i % 5] * relPos.x);
	}

	total = value;
	value = 0;

	for (; i < 10; i++)
	{
		value += Coeffs[i] * sin(freqs[i % 5] * relPos.y);
	}

	total += value;
	value = 0;

	for (; i < 15; i++)
	{
		value += Coeffs[i] * sin(freqs[i % 5] * relPos.z);
	}

	total += value;

	return (1.0f / 4.0f) * total * total;
	//return 1.0f; //TODO - we are assuimg cloud density of 1 for now.
	/*
	int LocalPointCount = 0;
	double TestDistSqu = VoxelHalfWidth * VoxelHalfWidth;
	for (int i = 0; i < TotalVoxels; i++)
	{
		for (int k = 0; k < Voxels[i].NumPoints; k++)
		{
			CartVec PointPos = Voxels[i].Points[k] + Voxels[i].GlobalPos;
			double DistSqu = (PointPos - pos).SquMag();
			if (DistSqu < TestDistSqu)
			{
				LocalPointCount += 1;
			}
		}
	}
	return LocalPointCount / 30.0f;*/
}

CloudVoxel* Cloud::GetVoxel(const CartVec& pos) const
{
	CartVec RelPos(pos - CornerPos);
	return Voxels + (int)(RelPos.x / VoxelWidth) + (int)(RelPos.y / VoxelWidth) * VoxelsWide + (int)(RelPos.z / VoxelWidth) * VoxelsWide_Squ;
}

//Find the point on the surface which is in the direction of view from pos
//outdist will be set to distance from the surface in that direction
CartVec Cloud::FindSurfacePoint(const CartVec& pos, const CartVec& view, double& outdist) const
{
	CartVec RelPos = pos - CentrePos;
	double dot = RelPos.Dot(view);
	outdist = sqrt(Radius_Squ + dot * dot - RelPos.SquMag()) - dot;
	return pos + view.Scale(outdist);
}