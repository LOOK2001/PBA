#ifndef __PBA_PBAUTILS_H__
#define __PBA_PBAUTILS_H__

#include "CollisionSurface.h"
#include "PbaThing.h"

#include <AABB.h>
extern std::vector<pba::AABB> testAABB;

using namespace std;

namespace pba {

	void AddCollisionSurface(pba::CollisionSurface& s, pba::PbaThing& p);

	void Display(pba::CollisionSurface& s);

	pba::CollisionSurface GenerateCollisionCube(double coeff, const Vector& trans = Vector(0.0));

	void combineCollisionSurface(pba::CollisionSurface& tar, pba::CollisionSurface& s2);

	double drand48();
};

#endif