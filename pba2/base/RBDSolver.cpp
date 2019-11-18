#include "RBDSolver.h"


pba::AdvanceRotationAndCOM::AdvanceRotationAndCOM(RigidBodyState& pq) :
	PQ(pq)
{}


void pba::AdvanceRotationAndCOM::solve(const double dt)
{
	double value = PQ->angular_velocity.magnitude();
	PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), -value * dt) * PQ->angular_rotation;
}

pba::AdvanceAngularVelocityAndVelocity::AdvanceAngularVelocityAndVelocity(RigidBodyState& pq, Force& f) :
	PQ(pq)
{
	tau = CreateTorqueFromForce(f);
}

void pba::AdvanceAngularVelocityAndVelocity::solve(const double dt)
{
	// Update angular velocity
	// 1. compute inertia
	PQ->recompute_MOI();

	// 2. compute torque
	tau->compute(PQ, dt);

	// 3. update angular velocity
	PQ->angular_velocity += (PQ->inverse_moi() * PQ->get_vector_attr("tau", 0) * dt);

	// Update linear velocity
	Vector center_of_force;
	for (int i = 0; i < PQ->nb(); i++)
		center_of_force += PQ->accel(i) * PQ->mass(i);

	PQ->linear_velocity += (center_of_force * dt) / PQ->totalmass();

	// Update velocity for each particles
	for (int i = 0; i < PQ->nb(); i++)
	{	
		Vector _u = PQ->angular_velocity ^ PQ->get_vector_attr("r", i);
		Vector vel = PQ->linear_velocity + _u;
		PQ->set_vel(i, vel);
	}
}

pba::GISolver pba::CreateAdvanceRotation(RigidBodyState& pq)
{
	return GISolver(new AdvanceRotationAndCOM(pq));
}

pba::GISolver pba::CreateAdvanceAngularVelocity(RigidBodyState& pq, Force& f)
{
	return GISolver(new AdvanceAngularVelocityAndVelocity(pq, f));
}


pba::AdvanceRotationWithCollisions::AdvanceRotationWithCollisions(RigidBodyState& pq, ElasticRBDCollisionHandler& coll):
	PQ(pq), CS(coll)
{}

void pba::AdvanceRotationWithCollisions::solve(const double dt)
{
	// Update Rotation
	double value = PQ->angular_velocity.magnitude();
	PQ->angular_rotation = rotation(PQ->angular_velocity.unitvector(), -value * dt)* PQ->angular_rotation;

	// Update Position
	PQ->center_of_mass += PQ->linear_velocity * dt;

	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector pos = PQ->center_of_mass + PQ->angular_rotation * PQ->get_vector_attr("p", i);
		PQ->set_pos(i, pos);

		PQ->set_attr("r", i, (PQ->angular_rotation * PQ->get_vector_attr("p", i)));
	}

	CS.handle_collisions(dt, PQ);
}

pba::GISolver pba::CreateAdvanceRotation(RigidBodyState& pq, ElasticRBDCollisionHandler& cs)
{
	return GISolver(new AdvanceRotationWithCollisions(pq, cs));
}