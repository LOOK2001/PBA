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
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int i = 0; i < tri_elements.size(); i++)
	{
		if (tri_elements[i]->hit(P, V, tmax, tc))
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[i];
				t.hit_index = i;
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[i];
				t.hit_index = i;
				t.status = true;
			}
		}
	}

	return t.status;
}

bool pba::CollisionSurfaceRaw::hit(const Vector& p, const Vector& v, const double r, const double tmax, CollisionData& t) const
{
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int j = 0; j < tri_elements.size(); j++)
	{
		if (tri_elements[j]->hit(p, v, r, tmax, tc))
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
			}
		}
	}

	return t.status;
}

bool pba::CollisionSurfaceRaw::hit(const SoftBodyState& s, const size_t i, const double tmax, CollisionData& t) const
{
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int j = 0; j < tri_elements.size(); j++)
	{
		if (tri_elements[j]->hit(s, i, tmax, tc))
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
			}
		}
	}

	return t.status;
}

bool pba::CollisionSurfaceRaw::hit(const RigidBodyState& s, const size_t i, const double tmax, CollisionData& t) const
{
	double tc = tmax;
	t.status = false;
	bool isFirst = true;

	// find all triangles that intersect
	for (int j = 0; j < tri_elements.size(); j++)
	{
		if (tri_elements[j]->hit(s, i, tmax, tc))
		{
			// find the largest backwards T (tc)
			if (isFirst) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
				isFirst = false;
			}
			else if (tc > t.t) {
				t.t = tc;
				t.tri = tri_elements[j];
				t.status = true;
			}
		}
	}

	return t.status;
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

void pba::CollisionSurfaceRaw::translate(const Vector& _trans)
{
	for (auto tri : tri_elements)
	{
		for (size_t i = 0; i < 3; i++)
		{
			tri->translate(_trans);
		}
	}
}

pba::CollisionSurface pba::makeCollisionSurface()
{
	return CollisionSurface(new CollisionSurfaceRaw);
}

