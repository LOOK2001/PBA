#include "AABB.h"
#include <assert.h>

pba::AABB::AABB(const Vector& lc, const Vector& uc)
{
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
		double newX = (llc.X() + urc.X()) * 0.5;
		aabb1 = AABB(llc, Vector(newX, urc.Y(), urc.Z()));
		aabb2 = AABB(Vector(newX, llc.Y(), llc.Z()), urc);
	}break;
	case 1:// Y
	{
		double newY = (llc.Y() + urc.Y()) * 0.5;
		aabb1 = AABB(llc, Vector(urc.X(), newY, urc.Z()));
		aabb2 = AABB(Vector(llc.X(), newY, llc.Z()), urc);
	}break;
	case 2:// Z
	{
		double newZ = (llc.Z() + urc.Z()) * 0.5;
		aabb1 = AABB(llc, Vector(urc.X(), urc.Y(), newZ));
		aabb2 = AABB(Vector(llc.X(), llc.Y(), newZ), urc);
	}break;
	}
}

const bool pba::AABB::isInside(const Vector& P) const
{
	pba::Vector min = llc;
	pba::Vector max = urc;

	return(P.X() >= min.X() && P.X() <= max.X() &&
		P.Y() >= min.Y() && P.Y() <= max.Y() &&
		P.Z() >= min.Z() && P.Z() <= max.Z());
}

const double pba::AABB::intersect(const Vector& start, const Vector& dir) const
{
	double tmin, tmax, tymin, tymax, tzmin, tzmax;

	Vector invdir = Vector(1 / dir.X(), 1 / dir.Y(), 1 / dir.Z());
	int sign[3];
	sign[0] = (invdir[0] < 0);
	sign[1] = (invdir[1] < 0);
	sign[2] = (invdir[2] < 0);

	Vector bounds[2]{ llc, urc };

	tmin = (bounds[sign[0]].X() - start.X()) * invdir.X();
	tmax = (bounds[1 - sign[0]].X() - start.X()) * invdir.X();
	tymin = (bounds[sign[1]].Y() - start.Y()) * invdir.Y();
	tymax = (bounds[1 - sign[1]].Y() - start.Y()) * invdir.Y();

	if (isinf(tmax))
		tmax = std::abs(tmax);
	if (isinf(tymax))
		tymax = std::abs(tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[sign[2]].Z() - start.Z()) * invdir.Z();
	tzmax = (bounds[1 - sign[2]].Z() - start.Z()) * invdir.Z();

	if (isinf(tzmax))
		tzmax = std::abs(tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	if (tmin < 0 && tmax < 0)
	{
		return tmin;
	}
	if (tmin > 0 && tmax > 0)
	{
		return tmin;
	}
	return tmax;
}
