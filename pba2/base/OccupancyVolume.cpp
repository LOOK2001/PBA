#include "OccupancyVolume.h"

pba::OccupancyVolume::OccupancyVolume(const AABB& aabb, const double h):
	bounds(aabb), cellsize(h)
{}

void pba::OccupancyVolume::set_cellsize(const double& v)
{
	cellsize = v;
}
