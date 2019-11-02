#include "CollisionHandler.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
	usetree = true;

	if (!c->triangle_size())
		return;

	Vector llc = c->get_triangle(0)->vertex(0), urc = c->get_triangle(0)->vertex(0);

	for (int i = 0; i < c->triangle_size(); i++) {
		for (int j = 0; j < 3; j++)	{

			const Vector& point = c->get_triangle(i)->vertex(j);

			for (int k = 0; k < 3; k++) {
				if (point[k] < llc[k])
					llc[k] = point[k];
				else if (point[k] > urc[k])
					urc[k] = point[k];
			}
		}
	}

	tree = new TraceTree(llc, urc, 0, 10, 4);
	tree->addObject(surf);
	tree->Divide();

	//surf->set_up_aabb();
}

void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& S)
{
	double vn;
	Vector vp, vr;
	if (!usetree)
	{
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, false, false, false, 0 };// = new pba::CollisionData;
			while (surf->hit(S->pos(i), S->vel(i), CD.t, CD))
			{
				Vector v = S->vel(i);
				Vector norm = CD.tri->N();
				vn = norm * S->vel(i);
				vp = S->vel(i) - norm * vn;
				vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

				// set new point
				Vector xc = S->pos(i) - (S->vel(i) * CD.t);
				Vector x = xc + vr * CD.t;
				S->set_pos(i, x);

				// set reflective velocity
				S->set_vel(i, vr);
 			}
		}
	}
	else
	{
#pragma omp parallel for
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, false, false, false, 0 };// = new pba::CollisionData;
			// check intersection with top bounding box
			while (tree->hit(S->pos(i), S->vel(i), CD.t, CD))
			{		
				Vector v = S->vel(i);
				Vector norm = CD.tri->N();
				vn = norm * S->vel(i);
				vp = S->vel(i) - norm * vn;
				vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

				// set new point
				Vector xc = S->pos(i) - (S->vel(i) * CD.t);
				Vector x = xc + vr * CD.t;
				S->set_pos(i, x);

				// set reflective velocity
				S->set_vel(i, vr);
			}
		}
	}
}
