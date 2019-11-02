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

#define INF 1.0e10
const double pba::AABB::intersect(const Vector& start, const Vector& dir) const
{
	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	tmin = 0.0f;
	tmax = INF;

	Vector invdir = Vector(1 / dir.X(), 1 / dir.Y(), 1 / dir.Z());
	int sign[3];
	sign[0] = (invdir[0] < 0);
	sign[1] = (invdir[1] < 0);
	sign[2] = (invdir[2] < 0);

	Vector bounds[2]{ llc, urc };
	
	//The plane perpendicular to x-axie  
	if (abs(dir.X()) < 0.0000001) //If the ray parallel to the plane  
	{
		//If the ray is not within AABB box, then not intersecting  
		if (start.X() < bounds[0].X() || start.X() > bounds[1].X())
			return false;
	}
	else
	{
		//Compute the distance of ray to the near plane and far plane  
		double ood = 1.0f / dir.X();
		double t1 = (bounds[0].X() - start.X()) * ood;
		double t2 = (bounds[1].X() - start.X()) * ood;

		//Make t1 be intersecting with the near plane, t2 with the far plane  
		if (t1 > t2)
		{
			double temp = t1;
			t1 = t2;
			t2 = temp;
		}

		//Compute the intersection of slab intersection intervals  
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;

		//Exit with no collision as soon as slab intersection becomes empty  
		if (tmin > tmax) return false;
	}// end for perpendicular to x-axie

	 //The plane perpendicular to y-axie  
	if (abs(dir.Y()) < 0.0000001) //If the ray parallel to the plane  
	{
		//If the ray is not within AABB box, then not intersecting  
		if (start.Y() < bounds[0].Y() || start.Y() > bounds[1].Y())
			return false;
	}
	else
	{
		//Compute the distance of ray to the near plane and far plane  
		double ood = 1.0f / dir.Y();
		double t1 = (bounds[0].Y() - start.Y()) * ood;
		double t2 = (bounds[1].Y() - start.Y()) * ood;

		//Make t1 be intersecting with the near plane, t2 with the far plane  
		if (t1 > t2)
		{
			double temp = t1;
			t1 = t2;
			t2 = temp;
		}

		//Compute the intersection of slab intersection intervals  
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;

		//Exit with no collision as soon as slab intersection becomes empty  
		if (tmin > tmax) return false;
	}// end for perpendicular to y-axie

	//The plane perpendicular to z-axie  
	if (abs(dir.Z()) < 0.0000001) //If the ray parallel to the plane  
	{
		//If the ray is not within AABB box, then not intersecting  
		if (start.Z() < bounds[0].Z() || start.Z() > bounds[1].Z())
			return false;
	}
	else
	{
		//Compute the distance of ray to the near plane and far plane  
		double ood = 1.0f / dir.Z();
		double t1 = (bounds[0].Z() - start.Z()) * ood;
		double t2 = (bounds[1].Z() - start.Z()) * ood;

		//Make t1 be intersecting with the near plane, t2 with the far plane  
		if (t1 > t2)
		{
			double temp = t1;
			t1 = t2;
			t2 = temp;
		}

		//Compute the intersection of slab intersection intervals  
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;

		//Exit with no collision as soon as slab intersection becomes empty  
		if (tmin > tmax) return false;
	}// end for perpendicular to z-axie

	//The plane perpendicular to z-axie  
	if (abs(dir.Z()) < 0.0000001) //If the ray parallel to the plane  
	{
		//If the ray is not within AABB box, then not intersecting  
		if (start.Z() < bounds[0].Z() || start.Z() > bounds[1].Z())
			return false;
	}
	else
	{
		//Compute the distance of ray to the near plane and far plane  
		double ood = 1.0f / dir.Z();
		double t1 = (bounds[0].Z() - start.Z()) * ood;
		double t2 = (bounds[1].Z() - start.Z()) * ood;

		//Make t1 be intersecting with the near plane, t2 with the far plane  
		if (t1 > t2)
		{
			double temp = t1;
			t1 = t2;
			t2 = temp;
		}

		//Compute the intersection of slab intersection intervals  
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;

		//Exit with no collision as soon as slab intersection becomes empty  
		if (tmin > tmax) return false;
	}// end for perpendicular to z-axie  

	if (tmin < 0 && tmax < 0)
	{
		return false;
	}
	if (tmin > 0 && tmax > 0)
	{
		return tmin;
	}
	return tmax;
}
