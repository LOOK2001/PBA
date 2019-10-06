#ifndef __PBA_PBAUTILS_H__
#define __PBA_PBAUTILS_H__

#include "CollisionSurface.h"
#include "PbaThing.h"

using namespace std;

namespace pba {

	void AddCollisionSurface(pba::CollisionSurface& s, pba::PbaThing& p);

	void Display(pba::CollisionSurface& s);

	pba::CollisionSurface GenerateCollisionCube(double coeff);

	void combineCollisionSurface(pba::CollisionSurface& tar, pba::CollisionSurface& s2);

	double drand48();
};

#endif