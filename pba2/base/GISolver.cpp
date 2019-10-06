#include "GISolver.h"

pba::GISolver pba::CreateForwardEulerSolver(GISolver& A, GISolver& B)
{
	return GISolver(new ForwardEulerSolver(A, B));
}

pba::GISolver pba::CreateLeapFrogSolver(GISolver& A, GISolver& B)
{
	return GISolver(new LeapFrogSolver(A, B));
}
