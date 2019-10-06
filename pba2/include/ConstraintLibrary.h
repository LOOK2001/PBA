#ifndef __PBA_CONSTRAINTLIBRARY_H__
#define __PBA_CONSTRAINTLIBRARY_H__

#include "Constraint.h"
#include "DynamicalState.h"

extern pba::Vector pointA;
extern pba::Vector pointB;
extern std::vector<std::pair<pba::Vector, pba::Vector>> lines;

namespace pba
{
	class ParticleOnSphereConstraint : public ConstraintBase
	{
	public:
		ParticleOnSphereConstraint(double radius, Vector& center, int particle_id) :
			R(radius),
			P(center),
			id(particle_id)
		{};
		~ParticleOnSphereConstraint() {};

		double compute(DynamicalState& s);
		Vector grad(DynamicalState& s, int index);
		Matrix gradgrad(DynamicalState& s, int i, int j);

		void solve(DynamicalState& s, double tol, int maxloop);

		void set_id(int d) { id = d; }
	
	private:
		double R;
		Vector P;
		int id;
	};
	Constraint CreateParticleOnSphereConstraint(double R, Vector cen, int p);
}

#endif // __PBA_CONSTRAINTLIBRARY_H__
