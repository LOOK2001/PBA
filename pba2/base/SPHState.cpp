#include "SPHState.h"
#include <assert.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

pba::SPHStateData::SPHStateData(const AABB& aabb, const double h, const std::string& nam /*= "SPHDataNoName"*/):
	DynamicalStateData(nam), OccupancyVolume(aabb, h), radius(2*h)
{
	create_attr("den", 0.0f);
}

pba::SPHStateData& pba::SPHStateData::operator=(const SPHStateData& d)
{
	radius = d.get_radius();
	t = d.t;
	nb_items = d.nb_items;
	name = d.name;
	int_attributes = d.int_attributes;
	float_attributes = d.float_attributes;
	vector_attributes = d.vector_attributes;
	color_attributes = d.color_attributes;
	re_find_main_attrs();
	return *this;
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
	// using OccupancyVolume to compute the density
	if (contents_nb() <= 0)
	{
		return;
	}

	std::vector<size_t> neighbors;
	std::vector<size_t> id_in_cell;

	for (size_t iii = 0; iii < contents_nb(); iii++)
	{
		get_id_from_contents(iii, id_in_cell);
		neighbor_cells(id_in_cell[0], id_in_cell[1], id_in_cell[2], neighbors);

		for (size_t jjj = 3; jjj < id_in_cell.size(); jjj++)
		{
			size_t id = id_in_cell[jjj];

			// compute A's density with each B 
			float density = 0.0f;

			for (int ii = 3; ii < id_in_cell.size(); ii++)
			{
				if (ii == id)
					continue;

				// compute density
				size_t j = id_in_cell[ii];
				float m = mass(j);
				Vector x = pos(j);
				float w = weight(id, x);
				density += (m * w);
			}

			// each neighbors
			for (int jj = 0; jj < neighbors.size(); jj++)
			{
				// get particles from each neighbors
				get_id_from_contents(jj, id_in_cell);

				// compute density A's density with each particles
				for (int kk = 3; kk < id_in_cell.size(); kk++)
				{
					// compute density
					size_t j = id_in_cell[kk];
					float m = mass(j);
					Vector x = pos(j);
					float w = weight(id, x);
					density += (m * w);
				}
			}
			set_attr("den", id, density);
		}
	}

// 	// all particles
// #pragma omp parallel for
// 	for (int i = 0; i < nb(); i++)
// 	{
// 		float density = 0.0f;
// #pragma omp parallel for
// 		for (int j = 0; j < nb(); j++)
// 		{
// 			if (j == i)
// 				continue;
// 
// 			float m = mass(j);
// 			Vector x = pos(j);
// 			float w = weight(i, x);
// 			density += (m * w);
// 		}
// 		set_attr("den", i, density);
// 	}
}

void pba::SPHStateData::populate()
{
 	DynamicalStateData pq = DynamicalStateData();
 	pq = *this;
	if (pq.nb() > 0)
	{
		OccupancyVolume::populate(pq);
	}
}

pba::SPHState pba::CreateSPH(const AABB& bounds, const double h, const std::string& nam /*= "SPHDataNoName"*/)
{
	return SPHState(new SPHStateData(bounds, h, nam));
}
