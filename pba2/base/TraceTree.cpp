#include "TraceTree.h"
#include <stack>

#include "PbaUtils.h"

pba::TraceTree::TraceTree(const Vector& llc, const Vector& urc, const int lvl, const int maxlvl, const int minobj) :
	aabb(llc, urc)
{
	level = lvl;
	max_levels = maxlvl;
	min_objects = minobj;
	node1 = node2 = nullptr;
}

pba::TraceTree::~TraceTree()
{
	if (node1)
		delete node1;
	if (node2)
		delete node2;

	object_list.clear();
}

void pba::TraceTree::Divide()
{
	if (level >= max_levels || object_list.size() <= min_objects || object_list.size() == 0)
	{
		return;
	}

	Vector _x;
	pba::AABB aabb1(_x, _x), aabb2(_x, _x);
	aabb.split(level % 3, aabb1, aabb2);

	node1 = new TraceTree(aabb1.LLC(), aabb1.URC(), level + 1, max_levels, min_objects);
	node2 = new TraceTree(aabb2.LLC(), aabb2.URC(), level + 1, max_levels, min_objects);

	if (level >= 10)
	{
// 		testAABB.push_back(aabb1);
// 		testAABB.push_back(aabb2);
	}

	// add triangles to divided box
	for (int i = object_list.size()-1; i >= 0; i--)
	{
		int tmpTri1 = -1, tmpTri2 = -1;
		for (int j = 0; j < 3; j++)
		{
			const Vector& _p = object_list[i]->vertex(j);

			if (!(tmpTri1 == i)){
				if (aabb1.isInside(_p)){
					node1->addObject(object_list[i]);
					tmpTri1 = i;
				}
			}

			if (!(tmpTri2 == i)){
				if (aabb2.isInside(_p)){
					node2->addObject(object_list[i]);
					tmpTri2 = i;
				}
			}
		}
		//object_list.pop_back();
	}

	node1->Divide();
	node2->Divide();
}

// void pba::TraceTree::Divide()
// {
// 	// if the point inside the box
// 	auto isPointInsideAABB = [](const AABB& aabb, const Vector& point)
// 	{
// 		pba::Vector min = aabb.LLC();
// 		pba::Vector max = aabb.URC();
// 
// 		return(point.X() >= min.X() && point.X() <= max.X() &&
// 			point.Y() >= min.Y() && point.Y() <= max.Y() &&
// 			point.Z() >= min.Z() && point.Z() <= max.Z());
// 	};
// 
// 	Vector _x;
// 	pba::AABB aabb1(_x, _x), aabb2(_x, _x);
// 	aabb.split(level % 3, aabb1, aabb2);
// 
// 	node1 = new TraceTree(aabb1.LLC(), aabb1.URC(), level+1, max_levels, min_objects);
// 	node2 = new TraceTree(aabb2.LLC(), aabb2.URC(), level+1, max_levels, min_objects);
// 
// 	// add triangles to divided box
// 	for (int i = 0; i < object_list.size(); i++)
// 	{
// // 		if (level == 3 && !flage) {
// // 			flage = true;
// // 			testAABB = aabb2;
// // 		}
// 
// 		int tmpTri1 = -1, tmpTri2 = -1;
// 		for (int j = 0; j < 3; j++)
// 		{
// 			const Vector& _p = object_list[i]->vertex(j);
// 
// 			if (!(tmpTri1 == i))
// 			{
// 				if (aabb1.isInside(_p))
// 				{
// 					node1->addObject(object_list[i]);
// 					tmpTri1 = i;
// 				}
// 			}
// 			if (!(tmpTri2 == i))
// 			{
// 				if (aabb2.isInside(_p))
// 				{
// 					node2->addObject(object_list[i]);
// 					tmpTri2 = i;
// 				}
// 			}
// 		}
// 		//object_list.pop_back();
// 	}
// 	if (level != 0)
// 	{
// 		//object_list.clear();
// 	}
// }

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
bool pba::TraceTree::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const
{
	t.status = false;

	std::stack<const pba::TraceTree*> stack;
	stack.push(this);
	while (!stack.empty())
	{
		const TraceTree* node = stack.top();
		stack.pop();
		if (!node)
			continue;

		if (!node->get_AABB().isInside(P - V * tmax))
			continue;

		if (!node->get_Node(1) && !node->get_Node(2))
		{
			// the node is leaf
			// until to get the deepest box,
			// check intersection with triangles
			double tc = tmax;
			bool isFirst = true;

			const std::vector<CollisionTriangle>& _objectList = node->get_ObjList();
			// find all triangles that intersect
			for (int i = 0; i < _objectList.size(); i++)
			{
				if (_objectList[i]->hit(P, V, tmax, tc))
				{
					// find the largest backwards T (tc)
					if (isFirst) {
						t.t = tc;
						t.tri = _objectList[i];
						t.hit_index = i;
						t.status = true;
						isFirst = false;
					}
					else if (tc > t.t) {
						t.t = tc;
						t.tri = _objectList[i];
						t.hit_index = i;
						t.status = true;
					}
				}
			}
			return t.status;
		}
		else
		{
			if(node->get_Node(1)) stack.push(node->get_Node(1));
			if(node->get_Node(2)) stack.push(node->get_Node(2));
		}
	}
	return false;


// 	t.status = false;
// 
// 	if (!aabb.isInside(P - V * tmax))
// 		return false;
// 
// 	if (node1) {
// 		if (node1->hit(P, V, tmax, t))
// 			return true;
// 	}	
// 	if (node2) {
// 		if (node2->hit(P, V, tmax, t))
// 			return true;
// 	}
// 
// 	// until to get the deepest box,
// 	// check intersection with triangles
// 	double tc = tmax;
// 	bool isFirst = true;
// 
// 	// find all triangles that intersect
// 	for (int i = 0; i < object_list.size(); i++)
// 	{
// 		if (object_list[i]->hit(P, V, tmax, tc))
// 		{
// 			// find the largest backwards T (tc)
// 			if (isFirst) {
// 				t.t = tc;
// 				t.tri = object_list[i];
// 				t.hit_index = i;
// 				t.status = true;
// 				isFirst = false;
// 			}
// 			else if (tc > t.t) {
// 				t.t = tc;
// 				t.tri = object_list[i];
// 				t.hit_index = i;
// 				t.status = true;
// 			}
// 		}
// 	}
// 	return t.status;
}

bool pba::TraceTree::hit(const RigidBodyState& s, const size_t i, const double tmax, CollisionData& t) const
{
	t.status = false;

	Matrix u = rotation(s->angular_velocity.unitvector(), s->angular_velocity.magnitude() * tmax) * s->angular_rotation;
	Vector x0 = s->center_of_mass - s->linear_velocity * tmax + u * s->get_vector_attr("p", i);
	Vector x1 = s->pos(i);

	if (!aabb.isInside(x0))
		return false;

	if (node1) {
		if (node1->hit(s, i, tmax, t))
			return true;
	}
	if (node2) {
		if (node2->hit(s, i, tmax, t))
			return true;
	}

	// until to get the deepest box,
	// check intersection with triangles
	double tc = tmax;
	bool isFirst = true;

	// find all triangles that intersect
	for (int i = 0; i < object_list.size(); i++)
	{
		if (object_list[i]->hit(s, i, tmax, tc))
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

// bool pba::TraceTree::hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const
// {
// 	double _t2 = aabb.intersect((P - (V * tmax)), V);
// 	t.status = false;
// 
// 	double tc = tmax;
// 	t.status = false;
// 	bool isFirst = true;
// 
// 	if (_t2 > 0 && _t2 <= 4*tmax) {
// 		// find all triangles that intersect
// 		for (int i = 0; i < object_list.size(); i++)
// 		{
// 			if (object_list[i]->hit(P, V, tmax, tc))
// 			{
// 				Vector test = P + (V * tmax);
// 
// 				// find the largest backwards T (tc)
// 				if (isFirst) {
// 					t.t = tc;
// 					t.tri = object_list[i];
// 					t.hit_index = i;
// 					t.status = true;
// 					isFirst = false;
// 				}
// 				else if (tc > t.t) {
// 					t.t = tc;
// 					t.tri = object_list[i];
// 					t.hit_index = i;
// 					t.status = true;
// 				}
// 			}
// 		}
// 	}
// 
// 	return t.status;
// 
// 	if (object_list.empty())
// 		return false;
// 
// 	double _t = aabb.intersect((P - V * tmax), V);
// 	t.status = false;
// 
// // 	if (level >= 4 && !flage)
// // 	{
// // 		flage = true;
// // 		for (int i = 0; i < object_list.size(); i++)
// // 		{
// // 			object_list[i]->set_color(Color(1.0, 0.5, 0.5, 1.0));
// // 		}
// // 	}
// 
// 	// intersection inside the segment
// 	if (_t > 0 && _t <= tmax) {
// 		double test = tmax - _t;
// 		// until to get the deepest box,
// 		// check intersection with triangles
// 		//if (level > max_levels || object_list.size() < min_objects) 
// 		{
// 			double tc = tmax;
// 			t.status = false;
// 			bool isFirst = true;
// 
// 			if (!flage) {
// 				flage = true;
// 				testAABB = aabb;
// 				for (int i = 0; i < object_list.size(); i++)
// 				{
// 					object_list[i]->set_color(Color(1.0, 1.0, 1.0, 1.0));
// 				}
// 			}
// 
// 			// find all triangles that intersect
// 			for (int i = 0; i < object_list.size(); i++)
// 			{
// 				if (object_list[i]->hit(P, V, tmax, tc))
// 				{
// 					// find the largest backwards T (tc)
// 					if (isFirst) {
// 						t.t = tc;
// 						t.tri = object_list[i];
// 						t.hit_index = i;
// 						t.status = true;
// 						isFirst = false;
// 					}
// 					else if (tc > t.t) {
// 						t.t = tc;
// 						t.tri = object_list[i];
// 						t.hit_index = i;
// 						t.status = true;
// 					}
// 				}
// 			}
// 			return t.status;
// 		}
// // 		else // check intersection with child bounding box 
// // 		{	
// // 			Divide();
// // 			if (node1->hit(P, V, tmax, t) || node2->hit(P, V, tmax, t))
// // 				return true;
// // 			else {
// // 				level = max_levels+1;
// // 				hit(P, V, tmax, t);
// // 			}
// //  				
// // // 			node1->Divide();
// // // 			node2->Divide();
// // 		}
// 	}
// 	else{
// 		t.status = false;
// 		return t.status;
// 	}
// }