#include "SPHState.h"
#include <assert.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

pba::SPHStateData::SPHStateData(const AABB& aabb, const double h, const std::string& nam /*= "SPHDataNoName"*/):
	DynamicalStateData(nam), OccupancyVolume(aabb, h), radius(2*h)
{
	create_attr("den", 0.0f);
}

void pba::SPHStateData::set_radius(const float& v)
{
	radius = v;
	set_cellsize(v / 2);
}

const float pba::SPHStateData::weight(size_t p, const Vector& P) const
{
	Vector x = pos(p);
	double r = (x - P).magnitude();
	double h = radius * 0.5;

	// Influence kernal: Cubic spline
	float res = (1.0 / (PI * pow(h, 3)));
	if (r >= 0 && r <= h)
		return res * (1.0 - 3.0 * pow((r / h), 2) + 0.75 * pow((r / h), 3));
	else if (r >= h && r <= 2*h)
		return res * (0.25 * pow((2 - (r / h)), 3));
	else if (r >= 2*h)
		return 0.0f;

	//assert(r < 0.0f);
}

const pba::Vector pba::SPHStateData::grad_weight(size_t p, const Vector& P) const
{
	Vector x = pos(p);
	double r = (x - P).magnitude();
	double h = radius * 0.5;

	float res = (1 / (PI * pow(h, 4)));
	if (r >= 0 && r <= h)
		res *= (-3 * (r / h) + 2.25 * (r / h));
	else if (r >= h && r <= 2 * h)
		res *= (-0.75 * pow((2 - (r / h)), 2));
	else if (r >= 2 * h)
		res = 0.0f;

	Vector test = (x - P) * res;
	if (!isnan(test.X()) || !isnan(test.Y()) || isnan(test.Z()))
	{
		int asd = 123;
	}
	return ((x - P)/ r) * res;

	//assert(r < 0.0f);
}

void pba::SPHStateData::compute_density()
{
	// all particles
	for (int i = 0; i < nb(); i++)
	{
		float density = 0.0f;
		for (int j = 0; j < nb(); j++)
		{
			if (j == i)
				continue;

			float m = mass(j);
			Vector x = pos(j);
			float w = weight(i, x);
			density += (m * w);
		}
		set_attr("den", i, density);
	}
}

pba::SPHState pba::CreateSPH(const AABB& bounds, const double h, const std::string& nam /*= "SPHDataNoName"*/)
{
	return SPHState(new SPHStateData(bounds, h, nam));
}
