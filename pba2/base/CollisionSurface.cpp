#include "CollisionSurface.h"

pba::CollisionSurfaceRaw::CollisionSurfaceRaw()
{
	coeff_of_restitution = 1.0f;
	coeff_of_sticky = 1.0f;
	visible = true;
	wireframe = false;
	tri_elements.clear();
}

void pba::CollisionSurfaceRaw::addTriangle(const CollisionTriangle& t)
{
	tri_elements.push_back(t);
}

bool pba::CollisionSurfaceRaw::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const
{
	// ตÝน้
// 	double tc = tmax;
// 	t.status = false;
// 
// 	double tmp = t.t;
// 
// 	int i = 0;
// 
// 	for (; i < tri_elements.size(); i++)
// 	{
// 		if (tri_elements[i]->hit(P, V, tmax, t.t))
// 		{
// 			if (t.t > tmp)
// 			{
// 				tmp = t.t;
// 				t.tri = tri_elements[i];
// 				t.hit_index = i;
// 				t.status = true;
// 			}
// 		}
// 	}
// 	tc = tmp;
// 	this->hit(P, V, tc, t);


	double tc = tmax;
	t.status = false;

	int i = 0;
	for (; i < tri_elements.size(); i++)
	{
		// 1. Compute where and when collision takes place
		// 2. If Xc is inside triangle, there is a collision
		if (tri_elements[i]->hit(P, V, tc, t.t))
		{
			if (tc == tmax)
			{
				tc = t.t;
				t.tri = tri_elements[i];
				t.hit_index = i;
				t.status = true;
			}
			else if (t.t > tc)
			{
				t.tri = tri_elements[i];
				t.status = true;
				t.hit_index = i;
				tc = t.t;
			}
		}
	}

	return t.status;
}

pba::CollisionSurface pba::makeCollisionSurface()
{
	return CollisionSurface(new CollisionSurfaceRaw);
}

