#include "OccupancyVolume.h"

pba::OccupancyVolume::OccupancyVolume(const AABB& aabb, const double h):
	bounds(aabb), cellsize(2*h)
{}

void pba::OccupancyVolume::populate(const DynamicalStateData& pq)
{
// 	for (int i = 0; i < 10; i++)
// 	{
// 		bounds.split();
// 	}
// 
// 	Vector 
// 	for (int i = 0; i < pq.nb(); i++)
// 	{
// 		Vector pos = pq.pos(i);
// 	}
}

void pba::OccupancyVolume::set_cellsize(const double& v)
{
	cellsize = v;
}
