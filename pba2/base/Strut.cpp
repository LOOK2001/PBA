#include "Strut.h"

void pba::Strut::compute(SoftBodyState& s)
{
	size_t p1 = edge->get_first_node(), p2 = edge->get_second_node();

	const Vector& a = s->pos(p1);
	const Vector& b = s->pos(p2);

	Vector FSab = -spring_constant * (a - b).unitvector() * ((a - b).magnitude() - edge->get_edge_length());
	Vector vel_ab = s->vel(p1) - s->vel(p2);
	Vector FFab = -friction_coeff * (a - b).unitvector() * ((a - b).unitvector() * vel_ab);

	Vector Ftotal = FSab + FFab;
	Vector aa = (Ftotal + s->accel(p1) * s->mass(p1)) / s->mass(p1);
	Vector ab = (s->accel(p2) * s->mass(p2) - Ftotal) / s->mass(p2);

	s->set_accel(p1, aa);
	s->set_accel(p2, ab);
}
