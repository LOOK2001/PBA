#include "TraceTree.h"

pba::TraceTree::TraceTree(const Vector& llc, const Vector& urc, const int lvl, const int maxlvl, const int minobj) :
	aabb(llc, urc)
{
	max_levels = maxlvl;
	min_objects = minobj;
}


void pba::TraceTree::Divide()
{
	// if the point inside the box
	auto isPointInsideAABB = [this](const Vector& point)
	{
		pba::Vector min = this->aabb.LLC();
		pba::Vector max = this->aabb.URC();

		return(point.X() >= min.X() && point.X() <= max.X() &&
			point.Y() >= min.Y() && point.Y() <= max.Y() &&
			point.Z() >= min.Z() && point.Z() <= max.Z());
	};
}

void pba::TraceTree::addObject(CollisionSurface& s)
{
	for (int i = 0; i < s->triangle_size(); i++)
	{
		object_list.push_back(s->get_triangle(i));
	}
}

bool pba::TraceTree::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const
{
	double tmin = aabb.intersect((P - V * tmax), V.unitvector());
	double dis = (V * tmax).magnitude();
	if (tmin >= tmax)
		return false;
	else
	{
		node1->Divide();
		node2->Divide();
		return (node1->hit(P, V, tmax, t) || node2->hit(P, V, tmax, t));
	}
}

void pba::TraceTree::addObject(CollisionTriangle& s)
{
	object_list.push_back(s);
}