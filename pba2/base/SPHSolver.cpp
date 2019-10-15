#include "SPHSolver.h"
#include "ForceLibrary.h"

pba::AdvanceSPHVelocity::AdvanceSPHVelocity(SPHState& pq, Force& f, float vclamp, float aclamp):
	PQ(pq), force(f), velocity_clamp(vclamp), acceleration_clamp(aclamp)
{}

void pba::AdvanceSPHVelocity::solve(const double dt)
{
	// update accel
	for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_accel(i, Vector(0.0, 0.0, 0.0));
	}

	std::shared_ptr<DynamicalStateData> pq = std::dynamic_pointer_cast<DynamicalStateData>(PQ);
	std::shared_ptr<AccumulatingForce> f = std::dynamic_pointer_cast<AccumulatingForce>(force);

	// External Force
	f->compute(pq, dt);
	// Pressure & Viscosity Force
	f->compute(PQ, dt);

	// update velocity
	for (int i = 0; i < PQ->nb(); i++)
	{
		if (PQ->accel(i).magnitude() > acceleration_clamp)
			PQ->set_accel(i, PQ->accel(i).unitvector() * acceleration_clamp);

		Vector _v = PQ->vel(i) + PQ->accel(i) * dt;
		if (_v.magnitude() > velocity_clamp)
			PQ->set_vel(i, _v.unitvector() * velocity_clamp);
		else
			PQ->set_vel(i, _v);
		//PQ->set_vel(i, PQ->vel(i) + PQ->accel(i) * dt);
	}
}

pba::GISolver pba::CreateAdvanceVelocity(SPHState& pq, Force& f, float vel_clamp, float accel_clamp)
{
	return GISolver(new AdvanceSPHVelocity(pq, f, vel_clamp, accel_clamp));
}

pba::AdvanceSPHPositionWithCollisions::AdvanceSPHPositionWithCollisions(SPHState& pq, ElasticCollisionHandler& coll):
	PQ(pq), CS(&coll)
{}

void pba::AdvanceSPHPositionWithCollisions::solve(const double dt)
{
	// update position
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector pos = PQ->pos(i) + PQ->vel(i) * dt;
		Vector vel = PQ->vel(i);
		double mag_pos = pos.magnitude();
		//assert(mag_pos > (-1000.0));
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	// handle collisions
	DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData>(PQ);
	CS->handle_collisions(dt, ss);

	// compute density
	PQ->compute_density();
}

pba::GISolver pba::CreateAdvancePosition(SPHState& pq, ElasticCollisionHandler& cs)
{
	return GISolver(new AdvanceSPHPositionWithCollisions(pq, cs));
}
