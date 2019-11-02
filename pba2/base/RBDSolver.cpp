#include "RBDSolver.h"


pba::AdvanceRotationAndCOM::AdvanceRotationAndCOM(RigidBodyState& pq) :
	PQ(pq)
{}


void pba::AdvanceRotationAndCOM::solve(const double dt)
{
	double value = PQ->angular_velocity.magnitude();
	PQ->angular_rotation = rotation(PQ->angular_velocity, -value * dt);
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

void pba::AdvanceRotationWithCollisions::solve(const double dt)
{
	// Update Rotation
	double value = PQ->angular_velocity.magnitude();
	PQ->angular_rotation = rotation(PQ->angular_velocity, -value * dt);

	// Update Position
	PQ->center_of_mass += PQ->linear_velocity * dt;

	for (int i = 0; i < PQ->nb(); i++)
	{
		Vector pos = PQ->center_of_mass + PQ->angular_rotation * PQ->pos(i);
		PQ->set_pos(i, pos);

		PQ->set_attr("r", i, (PQ->angular_rotation * pos));
	}
}