#include "CollisionHandler.h"
#include "LinearAlgebra.h"

void pba::CollisionHandler::set_collision_surface(CollisionSurface& c)
{
	surf = c;
	usetree = false;

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

void pba::ElasticRBDCollisionHandler::handle_collisions(const double dt, RigidBodyState& S)
{
	double vn;
	Vector vp, vr;
	bool isHit = false;

	if (!usetree)
	{
		pba::CollisionData CDLarg{ -dt, nullptr, false, false, false, 0 };
#pragma omp parallel for
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, false, false, false, i };// = new pba::CollisionData;
			if (surf->hit(S, i, CD.t, CD))
			{
				isHit = true;
				// find the largest backwards T (tc) for all particles and triangles
				if (CD.t > CDLarg.t)
				{
					CDLarg = CD;
				}
			}
		}

		if (isHit)
		{
			Vector norm = CDLarg.tri->N();
			double tmax = CDLarg.t;

			// 1. update rotation matrix
			Matrix u = rotation(S->angular_velocity, S->angular_velocity.magnitude() * tmax) * S->angular_rotation;

			// Solve for A
			Vector q = inverse(S->inertia_moment()) * (S->get_vector_attr("r", CDLarg.hit_index) ^ norm);
			double p0 = 2 * S->linear_velocity * norm;
			double p1 = S->angular_velocity * S->inertia_moment() * q;
			double p2 = q * S->inertia_moment() * S->angular_velocity;
			double p3 = 1 / S->totalmass();
			double p4 = q * S->inertia_moment() * q;
			double A = -(p0 + p1 + p2) / (p3 + p4);

			if (A == 0)
				return;

			// 2. update center of velocity
			Vector vr = S->linear_velocity + ((A * norm) / S->totalmass());

			// 3. update angular velocity
			Vector wr = S->angular_velocity + q * A;

			// 4. update center of mass
			Vector x = S->center_of_mass - S->linear_velocity * tmax + vr * tmax;

			// 5. update rotation matrix
			Matrix r = rotation(wr, -wr.magnitude() * tmax) * u;

			S->linear_velocity = vr;
			S->angular_velocity = wr;
			S->center_of_mass = x;
			S->angular_rotation = r;

			// 6. Update position and velocity
#pragma omp parallel for
			for (int i = 0; i < S->nb(); i++)
			{
				Vector pos = S->center_of_mass + S->angular_rotation * S->get_vector_attr("p", i);
				S->set_pos(i, pos);

				Vector r = S->angular_rotation * S->get_vector_attr("p", i);
				S->set_attr("r", i, r);

				Vector _u = S->angular_velocity ^ r;
				Vector vel = S->linear_velocity + _u;
				S->set_vel(i, vel);
			}

			handle_collisions(tmax, S);
		}
	}
	else
	{
		for (int i = 0; i < S->nb(); i++)
		{
			// check intersection with top bounding box
			pba::CollisionData CDLarg{ -dt, nullptr, false, false, false, 0 };
			for (int i = 0; i < S->nb(); i++)
			{
				pba::CollisionData CD{ dt, nullptr, false, false, false, i };// = new pba::CollisionData;
				if (tree->hit(S, i, CD.t, CD))
				{
					isHit = true;
					// find the largest backwards T (tc) for all particles and triangles
					if (CD.t > CDLarg.t)
					{
						CDLarg = CD;
					}
				}
			}
		}
	}
}
