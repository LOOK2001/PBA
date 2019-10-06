#include "Constraint.h"

pba::MultiConstraint pba::CreateMultiConstraint()
{
	return MultiConstraint(new MultiConstraintBase());
}

void pba::MultiConstraintBase::solve(DynamicalState& s, double tol, int maxloop)
{
	for (auto constraint : constraints)
	{
		constraint->solve(s, tol, maxloop);
	}
}
