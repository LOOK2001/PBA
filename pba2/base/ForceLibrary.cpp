#include "ForceLibrary.h"
#include "SPHForce.h"
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
	for (auto f : forces)
	{
		std::shared_ptr<SPHForce> SPHF = std::dynamic_pointer_cast<SPHForce>(f);
		if (SPHF)
			continue;
		f->compute(pq, dt);
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