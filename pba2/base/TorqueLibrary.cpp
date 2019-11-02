#include "TorqueLibrary.h"

void pba::TorqueFromForce::compute(RigidBodyState& s, const double dt)
{
	Vector tau;
	for (int i = 0; i < s->nb(); i++)
	{
		tau += s->get_vector_attr("r", i)^(s->mass(i)* s->accel(i));
	}
	s->set_attr("tau", 0, tau);
}

pba::Torque pba::CreateTorqueFromForce(Force& f)
{
	return Torque(new TorqueFromForce(f));
}

