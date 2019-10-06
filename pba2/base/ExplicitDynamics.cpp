#include <ExplicitDynamics.h>


pba::AdvancePosition::AdvancePosition(DynamicalState& pq)
{
	PQ = pq;
}

void pba::AdvancePosition::init() { return; }

void pba::AdvancePosition::solve(const double dt)
{
#pragma omp parallel for
	for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}
}

pba::AdvancedVelocity::AdvancedVelocity(DynamicalState& pq, Force& f)
{
	PQ = pq;
	force = f;
}

void pba::AdvancedVelocity::init() { return; }

void pba::AdvancedVelocity::solve(const double dt)
{
	force->compute(PQ, dt);

#pragma omp parallel for
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector v = PQ->vel(i);
		PQ->set_vel(i, PQ->vel(i) + PQ->accel(i) * dt);
		v = PQ->vel(i);
	}
}

pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq)
{
	return pba::GISolver(new pba::AdvancePosition(pq));
}

pba::GISolver pba::CreateAdvanceVelocity(DynamicalState& pq, Force& f)
{
	return pba::GISolver(new pba::AdvancedVelocity(pq, f));
}


/* AdvancePositionWithCollisions */
pba::AdvancePositionWithCollisions::AdvancePositionWithCollisions(DynamicalState& pq, CollisionHandler& coll)
{
	PQ = pq;
	CS = &coll;
}

void pba::AdvancePositionWithCollisions::init() { return; }

void pba::AdvancePositionWithCollisions::solve(const double dt)
{
#pragma omp parallel for
	for (int i = 0; i < PQ->nb(); i++)
	{
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	CS->handle_collisions(dt, PQ);
}

pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs)
{
	return GISolver(new AdvancePositionWithCollisions(pq, cs));
}


pba::AdvanceVelocityWithConstraint::AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f, MultiConstraint& c)
{
	PQ = pq;
	force = f;
	C = c;
}

void pba::AdvanceVelocityWithConstraint::solve(const double dt)
{
	// compute Gravity acceleration OR Boid Force acceleration
	force->compute(PQ, dt);

#pragma omp parallel for
	for (int i = 0; i < PQ->nb(); i++)	
	{
		 double Nsqr = C->compute(PQ, i);

		// compute the velocity by new accel
		Vector _v = PQ->vel(i) + PQ->accel(i) * dt;

		// v perpendicular to n
		Vector N = C->grad(PQ, i, i);
		_v = _v - ((_v * N) * N) / Nsqr;
		PQ->set_vel(i, _v);
	}
}

pba::GISolver pba::CreateAdvanceVelocity(DynamicalState& pq, Force& f, MultiConstraint& c)
{
	return pba::GISolver(new pba::AdvanceVelocityWithConstraint(pq, f, c));
}

pba::AdvancePositionWithConstraint::AdvancePositionWithConstraint(DynamicalState& pq, MultiConstraint& c)
{
	PQ = pq;
	C = c;
}

void pba::AdvancePositionWithConstraint::solve(const double dt)
{
	double tol = 1e-4;
	int maxloop = 20;
	std::vector<Vector> last;
	last.reserve(PQ->nb());

	// Updating the position for each particle
	for (int i = 0; i < PQ->nb(); i++)
	{
		// save the X0 for updating the velocity later
		last.push_back(PQ->pos(i));

		// update the position
		PQ->set_pos(i, PQ->pos(i) + PQ->vel(i) * dt);
	}

	// to update the position by Position-based method
	C->solve(PQ, tol, maxloop);

	// After Position-based method, to correct the velocity
	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector v = (PQ->pos(i) - last[i]) / dt;
		PQ->set_vel(i, v);
	}
}

pba::GISolver pba::CreateAdvancePosition(DynamicalState& pq, MultiConstraint& cs)
{
	return GISolver(new AdvancePositionWithConstraint(pq, cs));
}