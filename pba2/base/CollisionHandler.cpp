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

	tree = new TraceTree(llc, urc, 0, 5, 4);
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
			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
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
//#pragma omp parallel for
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
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

				CD.tri->set_hit_color(Color(1.0, 1.0, 1.0, 1.0));
			}
		}
	}
}

void pba::ElasticSphereCollisionHandler::handle_collisions(const double dt, DynamicalState& s)
{
	double vn;
	Vector vp, vr;
	if (!usetree)
	{
		for (int i = 0; i < s->nb(); i++)
		{
// 			float life = s->get_float_attr("life", i);
// 			if (life >= 1.0)
// 				continue;

			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
			float radius = s->get_float_attr("pscale", i);
			while (surf->hit(s->pos(i), s->vel(i), radius, CD.t, CD))
			{
				Vector v = s->vel(i);
				Vector norm = CD.tri->N();
				vn = norm * s->vel(i);
				vp = s->vel(i) - norm * vn;
				vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

				// set new point
				Vector xc = s->pos(i) - (s->vel(i) * CD.t);
				Vector x = xc + vr * CD.t;
				s->set_pos(i, x);

				// set reflective velocity
				s->set_vel(i, vr);
			}
		}

		for (size_t i = 0; i < s->nb(); i++)
		{
			for (size_t j = 0; j < s->nb(); j++)
			{
				if (j == i)
					continue;

// 				float life = s->get_float_attr("life", i);
// 				if (life >= 1.0)
// 					continue;
// 				life = s->get_float_attr("life", j);
// 				if (life >= 1.0)
// 					continue;

				float r1 = s->get_float_attr("pscale", i);
				float r2 = s->get_float_attr("pscale", j);
				Vector x1 = s->pos(i);
				Vector x2 = s->pos(j);
				double dis = (x1 - x2).magnitude();

				// 1. Detect collision
				if (dis >= (r1 + r2)) {
					continue;	// not touching or just touching
				}
				else if (dis < (r1 + r2)) {
					// overlapping
					Vector v1 = s->vel(i);
					Vector v2 = s->vel(j);
					Vector p1 = (x1 - x2);
					double p2 = (x1 - x2).magnitude();
					double p3 = (v1 - v2).magnitude();
					Vector p4 = (v1 - v2);
					double p5 = r1 + r2;
					double p6 = pow((p4 * p1), 2) - (p4 * p4) * (p2 * p2 - p5 * p5);
					double tmpT1 = (p4 * p1 + sqrt(p6)) / (p3 * p3);
					double tmpT2 = (p4 * p1 - sqrt(p6)) / (p3 * p3);
					double tc;

					// 2. Move backward in time by tc so that spheres touch
					if (tmpT1 > 0 && tmpT2 > 0)
						tc = (tmpT1 > tmpT2) ? tmpT2 : tmpT1;
					else
						tc = tmpT1;

					// 3. Change velocity to handle collision
					float m1 = s->mass(i);
					float m2 = s->mass(j);
					Vector n = (x1 - x2).unitvector();
					Vector dv = v1 - v2;
					Vector dvr = surf->coeff_sticky() * dv - (surf->coeff_sticky() + surf->coeff_restitution()) * n * (n * dv);
					Vector vcm = (m1 * x1 + m2 * x2) / (m1 + m2);
					Vector vr1 = vcm + (m2 / (m1 + m2) * dvr);
					Vector vr2 = vcm - (m1 / (m1 + m2) * dvr);
					Vector xc1 = x1 - v1 * tc + vr1 * tc;
					Vector xc2 = x2 - v2 * tc + vr2 * tc;

					// set new point
					s->set_pos(i, xc1);
					s->set_pos(j, xc2);

					// set reflective velocity
					s->set_vel(i, vr1);
					s->set_vel(j, vr2);
				}
			}
		}
	}
	else
	{}
}

void pba::ElasticSphereSphereCollisionHandler::handle_collisions(const double dt, DynamicalState& s)
{
	double vn;
	Vector vp, vr;
	pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
	if (!usetree)
	{
		for (size_t i = 0; i < s->nb(); i++)
		{
			for (size_t j = 0; j < s->nb(); j++)
			{
				if (j == i)
					continue;

				float r1 = s->get_float_attr("pscale", i);
				float r2 = s->get_float_attr("pscale", j);
				Vector x1 = s->pos(i);
				Vector x2 = s->pos(j);
				double dis = (x1 - x2).magnitude();

				// 1. Detect collision
				if (dis >= (r1 + r2)) {
					continue;	// not touching or just touching
				}
				else if (dis < (r1 + r2)) {
					// overlapping
					Vector v1 = s->vel(i);
					Vector v2 = s->vel(j);
					Vector p1 = (x1 - x2);
					double p2 = (x1 - x2).magnitude();
					double p3 = (v1 - v2).magnitude();
					Vector p4 = (v1 - v2);
					double p5 = r1 + r2;
					double p6 = pow((p4*p1), 2) - (p4 * p4) * (p2 * p2 - p5 * p5);
					double tmpT1 = (p4 * p1 + sqrt(p6)) / (p3 * p3);
					double tmpT2 = (p4 * p1 - sqrt(p6)) / (p3 * p3);
					double tc;

					// 2. Move backward in time by tc so that spheres touch
					if (tmpT1 > 0 && tmpT2 > 0)
						tc = (tmpT1 > tmpT2) ? tmpT2 : tmpT1;
					else
						tc = tmpT1;

					// 3. Change velocity to handle collision
					float m1 = s->mass(i);
					float m2 = s->mass(j);
					Vector n = (x1 - x2).unitvector();
					Vector dv = v1 - v2;
					Vector dvr = surf->coeff_sticky() * dv - (surf->coeff_sticky() + surf->coeff_restitution()) * n * (n * dv);
					Vector vcm = (m1 * x1 + m2 * x2) / (m1 + m2);
					Vector vr1 = vcm + (m2 / (m1 + m2) * dvr);
					Vector vr2 = vcm - (m1 / (m1 + m2) * dvr);
					Vector xc1 = x1 - v1*tc + vr1 * tc;
					Vector xc2 = x2 - v2*tc + vr2 * tc;

					// set new point
					s->set_pos(i, xc1);
					s->set_pos(j, xc2);

					// set reflective velocity
					s->set_vel(i, vr1);
					s->set_vel(j, vr2);
				}
			}
		}
	}
	else
	{
	}
}

void pba::ElasticRBDCollisionHandler::handle_collisions(const double dt, RigidBodyState& S)
{
	bool isHit = false;

	if (!usetree)
	{
		pba::CollisionData CDLarg{ -dt, nullptr, nullptr, false, false, false, 0 };
//#pragma omp parallel for
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, i };// = new pba::CollisionData;
			if (surf->hit(S, i, CD.t, CD))
			{
				isHit = true;
				// find the largest backwards T (tc) for all particles and triangles
				if (CD.t > CDLarg.t){
					CDLarg = CD;
				}
			}
		}

		if (isHit)
		{
			Vector norm = CDLarg.tri->N();
			double tmax = CDLarg.t;

			// 1. update U
			Matrix u = rotation(S->angular_velocity.unitvector(), S->angular_velocity.magnitude() * tmax) * S->angular_rotation;

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

			CDLarg.tri->set_hit_color(Color(1.0, 1.0, 1.0, 1.0));
			handle_collisions(tmax, S);
		}
	}
	else
	{
		for (int i = 0; i < S->nb(); i++)
		{
			// check intersection with top bounding box
			pba::CollisionData CDLarg{ -dt, nullptr, nullptr, false, false, false, 0 };
			for (int i = 0; i < S->nb(); i++)
			{
				pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, i };// = new pba::CollisionData;
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

void pba::ElasticSBDCollisionHandler::handle_collisions(const double dt, SoftBodyState& S)
{
	double vn;
	Vector vp, vr;
	if (!usetree)
	{
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
			while (surf->hit(S->pos(i), S->vel(i), CD.t, CD))
			{
				Vector v = S->vel(i);
				Vector norm = CD.tri->N();
				vn = norm * S->vel(i);
				vp = S->vel(i) - norm * vn;
				vr = (coeff_of_restitution * vp) - (0.1 * norm * vn);

				// set new point
				Vector xc = S->pos(i) - (S->vel(i) * CD.t);
				Vector x = xc + vr * CD.t;

				S->set_pos(i, x);

				// set reflective velocity
				S->set_vel(i, vr);

				//CD.tri->set_hit_color(Color(1.0, 1.0, 1.0, 1.0));
			}
		}
	}
	else
	{
		//#pragma omp parallel for
		for (int i = 0; i < S->nb(); i++)
		{
			pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
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

void pba::ElasticSBDSphereCollisionHandler::handle_collisions(const double dt, SoftBodyState& s)
{
	// SoftBody Collision
	ElasticSBDCollisionHandler::handle_collisions(dt, s);

	double vn;
	Vector vp, vr;
	// Sphere-triangle Collision
	for (int i = 0; i < s->nb(); i++)
	{
		pba::CollisionData CD{ dt, nullptr, nullptr, false, false, false, 0 };// = new pba::CollisionData;
		float radius = s->get_float_attr("pscale", i);

		while (surf->hit(s->pos(i), s->vel(i), radius, CD.t, CD))
		{
			if (CD.tri->get_collision_type() == CollisionTriangleRaw::TARGET)
			{
				PbaViewer* viewer = PbaViewer::Instance();
				viewer->sendMessage(PBA_HIT_TARGET);
			}
			Vector v = s->vel(i);
			Vector norm = CD.tri->N();
			vn = norm * s->vel(i);
			vp = s->vel(i) - norm * vn;
			vr = (surf->coeff_sticky() * vp) - (surf->coeff_restitution() * norm * vn);

			// set new point
			Vector xc = s->pos(i) - (s->vel(i) * CD.t);
			Vector x = xc + vr * CD.t;
			s->set_pos(i, x);

			// set reflective velocity
			s->set_vel(i, vr);
		}
	}

	// Sphere-Sphere Collision
	for (size_t i = 0; i < s->nb(); i++)
	{
		for (size_t j = 0; j < s->nb(); j++)
		{
			if (j == i)
				continue;

			float r1 = s->get_float_attr("pscale", i);
			float r2 = s->get_float_attr("pscale", j);
			Vector x1 = s->pos(i);
			Vector x2 = s->pos(j);
			double dis = (x1 - x2).magnitude();

			// 1. Detect collision
			if (dis >= (r1 + r2)) {
				continue;	// not touching or just touching
			}
			else if (dis < (r1 + r2)) {
				// overlapping
				Vector v1 = s->vel(i);
				Vector v2 = s->vel(j);
				Vector p1 = (x1 - x2);
				double p2 = (x1 - x2).magnitude();
				double p3 = (v1 - v2).magnitude();
				Vector p4 = (v1 - v2);
				double p5 = r1 + r2;
				double p6 = pow((p4 * p1), 2) - (p4 * p4) * (p2 * p2 - p5 * p5);
				double tmpT1 = (p4 * p1 + sqrt(p6)) / (p3 * p3);
				double tmpT2 = (p4 * p1 - sqrt(p6)) / (p3 * p3);
				double tc;

				// 2. Move backward in time by tc so that spheres touch
				if (tmpT1 > 0 && tmpT2 > 0)
					tc = (tmpT1 > tmpT2) ? tmpT2 : tmpT1;
				else
					tc = tmpT1;

				// 3. Change velocity to handle collision
				float m1 = s->mass(i);
				float m2 = s->mass(j);
				Vector n = (x1 - x2).unitvector();
				Vector dv = v1 - v2;
				Vector dvr = surf->coeff_sticky() * dv - (surf->coeff_sticky() + surf->coeff_restitution()) * n * (n * dv);
				Vector vcm = (m1 * x1 + m2 * x2) / (m1 + m2);
				Vector vr1 = vcm + (m2 / (m1 + m2) * dvr);
				Vector vr2 = vcm - (m1 / (m1 + m2) * dvr);
				Vector xc1 = x1 - v1 * tc + vr1 * tc;
				Vector xc2 = x2 - v2 * tc + vr2 * tc;

				// set new point
				s->set_pos(i, xc1);
				s->set_pos(j, xc2);

				// set reflective velocity
				s->set_vel(i, vr1);
				s->set_vel(j, vr2);
			}
		}
	}
}
