#include "CollisionHandler.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
	usetree = false;
}

void pba::ElasticCollisionHandler::handle_collisions(const double dt, DynamicalState& S)
{
	pba::CollisionData CD{0.0, nullptr, false, false, false, 0};// = new pba::CollisionData;
	double vn;
	Vector vp, vr;
	if (!usetree)
	{
		for (int i = 0; i < S->nb(); i++)
		{
			if (surf->hit(S->pos(i), S->vel(i), dt, CD))
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
		for (int i = 0; i < S->nb(); i++)
		{
			tree->addObject(surf);
			if (tree->hit(S->pos(i), S->vel(i), dt, CD))
			{

			}
		}
	}
}