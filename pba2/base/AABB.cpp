#include "AABB.h"
#include <assert.h>

pba::AABB::AABB(const Vector& lc, const Vector& uc)
{
	assert(lc < uc);
	llc = lc;
	urc = uc;
}

const bool pba::AABB::intersects(const AABB& b) const
{
	return(llc.X() <= b.urc.X() && urc.X() >= b.llc.X() &&
		llc.Y() <= b.urc.Y() && urc.Y() >= b.llc.Y() &&
		llc.Z() <= b.urc.Z() && urc.Z() >= b.llc.Z());
}

void pba::AABB::split(const int component, AABB& aabb1, AABB& aabb2) const
{
	switch (component)
	{
	case 0:// X
	{
		float newX = (llc.X() + urc.X()) * 0.5;
		aabb1 = AABB(llc, Vector(newX, urc.Y(), urc.Z()));
		aabb2 = AABB(Vector(newX, llc.Y(), llc.Z()), urc);
	}break;
	case 1:// Y
	{
		float newY = (llc.Y() + urc.Y()) * 0.5;
		aabb1 = AABB(llc, Vector(urc.X(), newY, urc.Z()));
		aabb2 = AABB(Vector(llc.X(), newY, llc.Z()), urc);
	}break;
	case 2:// Z
	{
		float newZ = (llc.Z() + urc.Z()) * 0.5;
		aabb1 = AABB(llc, Vector(urc.X(), urc.Y(), newZ));
		aabb2 = AABB(Vector(llc.X(), llc.Y(), newZ), urc);
	}break;
	}
}

const double pba::AABB::intersect(const Vector& start, const Vector& dir) const
{
	double tmin, tmax, tymin, tymax, tzmin, tzmax, divx, divy, divz;
	divx = 1 / dir.X();
	divy = 1 / dir.Y();
	divz = 1 / dir.Z();
	if (divx >= 0){
		tmin = (llc.X() - start.X()) * divx;
		tmax = (urc.X() - start.X()) * divx;
	}
	else {
		tmin = (urc.X() - start.X()) * divx;
		tmax = (llc.X() - start.X()) * divx;
	}
	if (divy >= 0)
	{
		tymin = (llc.Y() - start.Y()) * divy;
		tymax = (urc.Y() - start.Y()) * divy;
	}
	else {
		tymin = (urc.Y() - start.Y()) * divy;
		tymax = (llc.Y() - start.Y()) * divy;
	}
	if ((tmin > tymax) || (tymin > tmax))
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;
	if (divz >= 0){
		tzmin = (llc.Z() - start.Z()) * divz;
		tzmax = (urc.Z() - start.Z()) * divz;
	}
	else {
		tzmin = (urc.Z() - start.Z()) * divz;
		tzmax = (llc.Z() - start.Z()) * divz;
	}
	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	return tmin;
}
