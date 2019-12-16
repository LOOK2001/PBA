#include "SBDSolver.h"

void pba::AdvanceSoftBodyPosition::solve(const double dt)
{
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		Vector _vel = PQ->vel(i) + PQ->accel(i) * dt;
		PQ->set_vel(i, _vel);
	}
}

void pba::AdvanceSoftBodyVelocity::solve(const double dt)
{
	for (size_t i = 0; i < PQ->nb(); i++)
		PQ->set_accel(i, Vector(0.0, 0.0, 0.0));

	force->compute(PQ, dt);

	for (size_t i = 0; i < PQ->nb(); i++)
	{
		Vector _vel = PQ->vel(i) + PQ->accel(i) * dt;
		PQ->set_vel(i, _vel);
	}

	for (auto fixed : m_fixedId) {
		PQ->set_vel(fixed, Vector(0.0, 0.0, 0.0));
	}
}

pba::GISolver pba::CreateAdvanceRotation(SoftBodyState& pq)
{
	return GISolver(new AdvanceSoftBodyPosition(pq));
}

pba::GISolver pba::CreateAdvanceAngularVelocity(SoftBodyState& pq, Force& f)
{
	return GISolver(new AdvanceSoftBodyVelocity(pq, f));
}

void pba::AdvanceSoftBodyPositionWithCollisions::solve(const double dt)
{
	for (size_t i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	CS.handle_collisions(dt, PQ);
}

pba::GISolver pba::CreateAdvanceRotation(SoftBodyState& pq, ElasticSBDCollisionHandler& cs)
{
	return GISolver(new AdvanceSoftBodyPositionWithCollisions(pq, cs));
}