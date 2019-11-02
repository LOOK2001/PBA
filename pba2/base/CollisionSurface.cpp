#include "CollisionSurface.h"

pba::CollisionSurfaceRaw::CollisionSurfaceRaw():
	aa_bb(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))
{
	coeff_of_restitution = 1.0;
	coeff_of_sticky = 1.0;
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
// 	double _t = aa_bb.intersect((P - V*tmax), V);
// 
// 	// intersection inside the segment
// 	if (!(_t > 0 && _t <= tmax))
// 		return false;

	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int i = 0; i < tri_elements.size(); i++)
	{
		if (tri_elements[i]->hit(P, V, tmax, tc))
		{
			Vector test = P + (V * tmax);

			// find the largest backwards T (tc)
			if (isFirst){
				t.t = tc;
				t.tri = tri_elements[i];
				t.hit_index = i;
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t){
				t.t = tc;
				t.tri = tri_elements[i];
				t.hit_index = i;
				t.status = true;
			}
		}
	}

	return t.status;


// 	double tc = tmax;
// 	t.status = false;
// 
// 	int i = 0;
// 	for (; i < tri_elements.size(); i++)
// 	{
// 		// 1. Compute where and when collision takes place
// 		// 2. If Xc is inside triangle, there is a collision
// 		if (tri_elements[i]->hit(P, V, tc, t.t))
// 		{
// 			if (tc == tmax)
// 			{
// 				tc = t.t;
// 				t.tri = tri_elements[i];
// 				t.hit_index = i;
// 				t.status = true;
// 			}
// 			else if (t.t > tc)
// 			{
// 				t.tri = tri_elements[i];
// 				t.status = true;
// 				t.hit_index = i;
// 				tc = t.t;
// 			}
// 		}
// 	}
// 
// 	return t.status;
}

void pba::CollisionSurfaceRaw::set_up_aabb()
{
	Vector llc = tri_elements[0]->vertex(0), urc = tri_elements[0]->vertex(0);

	for (int i = 0; i < tri_elements.size(); i++) {
		for (int j = 0; j < 3; j++) {

			const Vector& point = tri_elements[i]->vertex(j);

			for (int k = 0; k < 3; k++) {
				if (point[k] < llc[k])
					llc[k] = point[k];
				else if (point[k] > urc[k])
					urc[k] = point[k];
			}
		}
	}
	aa_bb = AABB(llc, urc);
}

pba::CollisionSurface pba::makeCollisionSurface()
{
	return CollisionSurface(new CollisionSurfaceRaw);
}

