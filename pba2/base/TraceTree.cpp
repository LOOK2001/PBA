#include "TraceTree.h"

pba::TraceTree::TraceTree(const Vector& llc, const Vector& urc, const int lvl, const int maxlvl, const int minobj) :
	aabb(llc, urc)
{
	level = lvl;
	max_levels = maxlvl;
	min_objects = minobj;
}


pba::TraceTree::~TraceTree()
{
	if (node1)
		delete node1;
	if (node2)
		delete node2;
}

void pba::TraceTree::Divide()
{
	// if the point inside the box
	auto isPointInsideAABB = [](const AABB& aabb, const Vector& point)
	{
		pba::Vector min = aabb.LLC();
		pba::Vector max = aabb.URC();

		return(point.X() >= min.X() && point.X() <= max.X() &&
			point.Y() >= min.Y() && point.Y() <= max.Y() &&
			point.Z() >= min.Z() && point.Z() <= max.Z());
	};

	Vector _x;
	pba::AABB aabb1(_x, _x), aabb2(_x, _x);
	aabb.split(level % 3, aabb1, aabb2);

	node1 = new TraceTree(aabb1.LLC(), aabb1.URC(), level+1, max_levels, min_objects);
	node2 = new TraceTree(aabb2.LLC(), aabb2.URC(), level+1, max_levels, min_objects);

	// add triangles to divided box
	for (int i = object_list.size()-1; i >= 0; i--)
	{
		int tmpTri1 = -1, tmpTri2 = -1;
		for (int j = 0; j < 3; j++)
		{
			const Vector& _p = object_list[i]->vertex(j);
			if (aabb1.isInside(_p))
			{
				if (!(tmpTri1 == i))
				{
					node1->addObject(object_list[i]);
					tmpTri1 = i;
				}
			}
			if (aabb2.isInside(_p))
			{
				if (!(tmpTri2 == i))
				{
					node2->addObject(object_list[i]);
					tmpTri2 = i;
				}	
			}
		}
		//object_list.pop_back();
	}
	if (level != 0)
	{
		object_list.clear();
	}
}

void pba::TraceTree::addObject(CollisionSurface& s)
{
	for (int i = 0; i < s->triangle_size(); i++)
	{
		object_list.push_back(s->get_triangle(i));
	}
}

void pba::TraceTree::addObject(CollisionTriangle& s)
{
	object_list.push_back(s);
}

bool pba::TraceTree::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t)
{

	double _t = aabb.intersect((P - V * tmax), V);
	t.status = false;

	// intersection inside the segment
	if (_t >= 0 && _t <= tmax) {
		// until to get the deepest box,
		// check intersection with triangles
		if (level > max_levels || object_list.size() < min_objects) 
		{
			double tc = tmax;
			t.status = false;
			bool isFirst = true;

			// find all triangles that intersect
			for (int i = 0; i < object_list.size(); i++)
			{
				if (object_list[i]->hit(P, V, tmax, tc))
				{
					// find the largest backwards T (tc)
					if (isFirst) {
						t.t = tc;
						t.tri = object_list[i];
						t.hit_index = i;
						t.status = true;
						isFirst = false;
					}
					else if (tc > t.t) {
						t.t = tc;
						t.tri = object_list[i];
						t.hit_index = i;
						t.status = true;
					}
				}
			}
			return t.status;
		}
		else // check intersection with child bounding box 
		{	
			Divide();
			if (!node1->hit(P, V, tmax, t) && !node2->hit(P, V, tmax, t))
				return false;
// 			node1->Divide();
// 			node2->Divide();
		}
	}
	else{
		t.status = false;
		return t.status;
	}
}