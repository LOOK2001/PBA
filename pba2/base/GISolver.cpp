#include "GISolver.h"

pba::GISolver pba::CreateGISolverSubstep(GISolver& s, int nbsteps)
{
	return GISolver(new GISolverSubstep(s, nbsteps));
}

pba::GISolver pba::CreateForwardEulerSolver(GISolver& A, GISolver& B)
{
	return GISolver(new ForwardEulerSolver(A, B));
}

pba::GISolver pba::CreateLeapFrogSolver(GISolver& A, GISolver& B)
{
	return GISolver(new LeapFrogSolver(A, B));
}
