#include "ForceLibrary.h"
#include "SPHForce.h"
#include "Strut.h"
#include "PbaUtils.h"

void pba::SimpleGravityForce::compute(pba::DynamicalState& pq, const double dt)
{
#pragma omp parallel for
	for (int i = 0; i < pq->nb(); i++)
	{
		//pba::Vector acc = G / pq->mass(i);
		pq->set_accel(i, G / pq->mass(i));
	}
}

pba::Force pba::CreateSimpleGravityForce(const Vector& G)
{
	return pba::Force(new pba::SimpleGravityForce(G));
}

void pba::AccumulatingForce::compute(pba::DynamicalState& pq, const double dt)
{
	for (auto f : forces){
		f->compute(pq, dt);
	}
}

void pba::AccumulatingForce::compute(RigidBodyState& s, const double dt)
{
	for (auto f : forces) {
		f->compute(s, dt);
	}
}

void pba::AccumulatingForce::compute(SoftBodyState& s, const double dt)
{
	for (auto f : forces){
		f->compute(s, dt);
	}
}

void pba::AccumulatingForce::compute(pba::SPHState& s, const double dt)
{
	for (auto f : forces)
	{
		std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		if (!SPHF)
			continue;
		SPHF->compute(s, dt);
	}
}

void pba::AccumulatingForce::add(Force& f)
{
	forces.push_back(f);
}

pba::Force pba::CreateAccumulatingForce()
{
	return pba::Force(new pba::AccumulatingForce);
}

void pba::AccumulatingBoidForce::compute(pba::DynamicalState& pq, const double dt)
{
	for (int i = 0; i< pq->nb(); i++)
	{
		double kr = 0, kf = 0;
		Vector a_avoid, a_velMat, a_center;
		double _amax = amax;

		for (int j = 0; j < pq->nb(); j++)
		{
			if (j == i)
				continue;

			Vector xa = pq->pos(i);
			Vector xb = pq->pos(j);
			Vector va = pq->vel(i);
			Vector vb = pq->vel(j);

			// Influence Range
			double r = (xa - xb).magnitude();

			if (r < range)
				kr = 1;
			else if (r > range && r < range_ramp)
				kr = (range_ramp - r) / (range_ramp - range);
			else if (r > range_ramp)
				kr = 0;

			// Influence FOV
			double t = (xb - xa).unitvector() * va.unitvector();

			if (t > cosfovshell)
				kf = 1;
			else if (t > cosfov && t < cosfovshell)
				kf = (cosfov - t) / (cosfov - cosfovshell);
			else if (t < cosfov)
				kf = 0;

			// Avoidance
			a_avoid += (A * (xa - xb).unitvector() * (1 / (xa - xb).magnitude()) * kr * kf);

			// Velocity Matching
			a_velMat += (V * (vb - va) * kr * kf);

			// Centering
			a_center += (C * (xb - xa) * kr * kf);

		}// particle b

		// Acceleration Prioritization
		double a_len = a_avoid.magnitude();
		if (a_len > _amax) {
			a_avoid = _amax * a_avoid.unitvector();
			a_velMat = a_center = Vector(0.0, 0.0, 0.0);
		}
		else {
			_amax = _amax - a_len;
			a_len = a_velMat.magnitude();

			if (a_len > _amax) {
				a_velMat = _amax * a_velMat.unitvector();
				a_center = Vector(0.0, 0.0, 0.0);
			}
			else {
				_amax = _amax - a_len;
				a_len = a_center.magnitude();

				if (a_len > _amax) {
					a_center = _amax * a_center.unitvector();
				}
			}
		}
		Vector aTotal = a_avoid + a_velMat + a_center;

		// if the boid is the lead boid
		if (i == leadBoidID) {
			Vector dir(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
			aTotal = dir.unitvector() * aTotal.magnitude();
		}

		pq->set_accel(i, aTotal);

	} // particle a
}

pba::Force pba::CreateAccumulatingBoidForce(const double A, const double V, const double C, const double Max, const double range, const double range_ramp)
{
	return pba::Force(new pba::AccumulatingBoidForce(A, V, C, Max, range, range_ramp));
}


void pba::AccumulatingRandomBoidForce::compute(pba::DynamicalState& pq, const double dt)
{
	Vector a;
#pragma omp parallel for
	for (int i = 0; i < pq->nb(); i++)
	{
		Vector a(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
		pq->set_accel(i, a);
	}
}

pba::Force pba::CreateAccumulatingRandomBoidForce()
{
	return pba::Force(new pba::AccumulatingRandomBoidForce());
}

void pba::AccumulatingGravityForce::compute(pba::DynamicalState& pq, const double dt)
{
//#pragma omp parallel for
	for (int i = 0; i < pq->nb(); i++)
	{
		Vector a = pq->accel(i);
		pq->set_accel(i, pq->accel(i) + (G / pq->mass(i)));
		a = pq->accel(i);
	}
}

pba::Force pba::CreateAccumulatingGravityForce(const Vector& G)
{
	return Force(new AccumulatingGravityForce(G));
}

void pba::AccumulatingStrutForce::compute(pba::SoftBodyState& pq, const double dt)
{
	for (size_t i = 0; i < pq->nb_pairs(); i++)
	{
		const pba::SoftEdge& edge = pq->get_connected_pair(i);
		const size_t& p1 = edge->get_first_node();
		const size_t& p2 = edge->get_second_node();

		const Vector& a = pq->pos(p1);
		const Vector& b = pq->pos(p2);

		Vector dir_ab = (a - b).unitvector();
		Vector fs = -spring * dir_ab * ((a - b).magnitude() - edge->get_edge_length());
		Vector vel_ab = pq->vel(p1) - pq->vel(p2);
		Vector ff = -friction * dir_ab * ((a - b).unitvector() * vel_ab);

		Vector Ftotal = fs + ff;
		Vector aa = (Ftotal + pq->accel(p1) * pq->mass(p1)) / pq->mass(p1);
		Vector ab = (pq->accel(p2) * pq->mass(p2) - Ftotal) / pq->mass(p2);

// 		if ((a - b).magnitude() > pq->get_edge_threshold())
// 			return;

		pq->set_accel(p1, aa);
		pq->set_accel(p2, ab);
	}
}

pba::Force pba::CreateAccumulatingStrutForce(const double G, const double f)
{
	return Force(new AccumulatingStrutForce(G, f));
}

void pba::AccumulatingStrutAreaForce::compute(pba::SoftBodyState& pq, const double dt)
{
	for (size_t ii = 0; ii < pq->nb_area_sets(); ii++)
	{
		size_t i = pq->get_area_set(ii)->get_first_node();
		size_t j = pq->get_area_set(ii)->get_second_node();
		size_t k = pq->get_area_set(ii)->get_third_node();

		double A0 = pq->get_area_set(ii)->get_area();
		Vector p0 = pq->pos(i);
		Vector p1 = pq->pos(j);
		Vector p2 = pq->pos(k);
		Vector d0 = (p0 - 0.5 * (p1 + p2)).unitvector();
		Vector d1 = (p1 - 0.5 * (p0 + p2)).unitvector();
		Vector d2 = (p2 - 0.5 * (p0 + p1)).unitvector();
		Vector v0 = pq->vel(i);
		Vector v1 = pq->vel(j);
		Vector v2 = pq->vel(k);
		Vector V0 = v0 - 0.5 * (v1 + v2);
		Vector V1 = v1 - 0.5 * (v0 + v2);
		Vector V2 = v2 - 0.5 * (v0 + v1);
		double A = ((p1 - p0) ^ (p2 - p0)).magnitude();

		Vector f0 = -spring * d0 * (A - A0) - friction * d0 * (d0 * V0);
		Vector f1 = -spring * d1 * (A - A0) - friction * d1 * (d1 * V1);
		Vector f2 = -spring * d2 * (A - A0) - friction * d2 * (d2 * V2);

		f0 += (pq->accel(i) * pq->mass(i));
		f1 += (pq->accel(j) * pq->mass(j));
		f2 += (pq->accel(k) * pq->mass(k));

		pq->set_accel(i, f0 / pq->mass(i));
		pq->set_accel(j, f1 / pq->mass(j));
		pq->set_accel(k, f2 / pq->mass(k));
	}
}

pba::Force pba::CreateAccumulatingStrutAreaForce(const double G, const double f)
{
	return Force(new AccumulatingStrutAreaForce(G, f));
}

void pba::AccumulatingStrutBendForce::compute(pba::SoftBodyState& pq, const double dt)
{
	for (size_t ii = 0; ii < pq->nb_bendables(); ii++)
	{
		size_t i = pq->get_bendable(ii)->get_first_node();
		size_t j = pq->get_bendable(ii)->get_second_node();
		size_t k = pq->get_bendable(ii)->get_third_node();
		size_t l = pq->get_bendable(ii)->get_fourth_node();

		const SoftBendable& sb = pq->get_bendable(ii);
		double t0 = sb->get_value();

		Vector e1 = pq->pos(j) - pq->pos(i);
		Vector e2 = pq->pos(k) - pq->pos(i);
		Vector n0 = (e1 ^ e2).unitvector();
		Vector f1 = pq->pos(j) - pq->pos(l);
		Vector f2 = pq->pos(k) - pq->pos(l);
		Vector n1 = (f2 ^ f1).unitvector();
		Vector h = (pq->pos(k) - pq->pos(j)).unitvector();

		double sint = h * (n0 ^ n1);
		double cost = n1 * n0;
		double t = atan2(sint, cost);

		// force on i and l
		Vector F0 = spring * (t - t0) * n0;
		Vector F3 = spring * (t - t0) * n1;

		// calculate center of mass
		float total_mass = pq->mass(i) + pq->mass(j) + pq->mass(k) + pq->mass(l);
		Vector p1 = pq->mass(i) * pq->pos(i);
		Vector p2 = pq->mass(j) * pq->pos(j);
		Vector p3 = pq->mass(k) * pq->pos(k);
		Vector p4 = pq->mass(l) * pq->pos(l);
		Vector xcm = (p1 + p2 + p3 + p4) / total_mass;

		Vector r0 = pq->pos(i) - xcm;
		Vector r1 = pq->pos(j) - xcm;
		Vector r2 = pq->pos(k) - xcm;
		Vector r3 = pq->pos(l) - xcm;

		// to solve force on j and k
		double q0 = ((r1 - r2) * (r2 - r0)) / (pow((r1 - r2).magnitude(), 2));
		double q3 = ((r1 - r2) * (r2 - r3)) / (pow((r1 - r2).magnitude(), 2));

		Vector F1 = q0 * F0 + q3 * F3;
		Vector F2 = -(q0 + 1) * F0 - (q3 + 1) * F3;

		// recalculate force on i, j, k and j
		F0 += (pq->accel(i) * pq->mass(i));
		F1 += (pq->accel(j) * pq->mass(j));
		F2 += (pq->accel(k) * pq->mass(k));
		F3 += (pq->accel(l) * pq->mass(l));

		// set accel
		pq->set_accel(i, F0 / pq->mass(i));
		pq->set_accel(j, F1 / pq->mass(j));
		pq->set_accel(k, F2 / pq->mass(k));
		pq->set_accel(l, F3 / pq->mass(l));
	}
}

pba::Force pba::CreateAccumulatingStrutBendForce(const double G, const double f)
{
	return Force(new AccumulatingStrutBendForce(G, f));
}
