#include "ConstraintLibrary.h"

pba::Vector pointA;
pba::Vector pointB;
std::vector<std::pair<pba::Vector, pba::Vector>> lines;

double pba::ParticleOnSphereConstraint::compute(DynamicalState& s)
{
	Vector a = s->accel(id);
	Vector V = s->vel(id);
	Vector N = grad(s, id);
	double Nsqr = N * N;
	Matrix mat = gradgrad(s, 0, 0);

	double C = (N * N) - (R * R);

	double Ks = ConstraintBase::get_Ks();
	double Kf = ConstraintBase::get_Kf();

	// to compute the new velocity based on Relaxation method
	Vector _a = a - (((N * a) / Nsqr)) * N - ((((V * mat * V) - Ks * C - Kf * V * N)/ Nsqr) * N);
	s->set_accel(id, _a);
	
	return (Nsqr);
}

pba::Vector pba::ParticleOnSphereConstraint::grad(DynamicalState& s, int index)
{
	Vector x = s->pos(index);
	Vector n = x - P;
	return n;
}

pba::Matrix pba::ParticleOnSphereConstraint::gradgrad(DynamicalState& s, int i, int j)
{
	Matrix mat = Matrix();
	for (int i = 0, j = 0; i < 3; i++, j++) {
		mat[i][j] = 1;
	}
	return mat;
}

void pba::ParticleOnSphereConstraint::solve(DynamicalState& s, double tol, int maxloop)
{
	// Position-Based
	Vector x0 = s->pos(id), x1;
	Vector N1 = grad(s, id), N2;
	double Nsqr1 = N1 * N1, Nsqr2;

	double C;

	// loop to find the small dX
	for (int i = 0; i < maxloop; i++)
	{
		C = ((N1 * N1) - (R * R));

		x1 = x0 - ((C * N1) / Nsqr1);
		N2 = (x1 - P);
		Nsqr2 = N2 * N2;

		if ((x1 - x0).magnitude() < tol)
			break;

		N1 = N2;
		Nsqr1 = Nsqr2;
		x0 = x1;
	}

	// get the new position by Position-based method
	s->set_pos(id, x1);

#ifndef _DEBUG
	// Debug
	lines.reserve(2);
	pointA = P;
	pointB = P + grad(s, id);
	lines.push_back(std::make_pair(P, N2));
	lines.push_back(std::make_pair(P, P + grad(s, id)));
#endif // _DEBUG
}

pba::Constraint pba::CreateParticleOnSphereConstraint(double R, Vector cen, int p)
{
	return Constraint(new ParticleOnSphereConstraint(R, cen, p));
}

