#ifndef __PBA_GISOLVER_H__
#define __PBA_GISOLVER_H__

#include <cmath>
#include <memory>

namespace pba{
	class GISolverBase
	{
	public:
		GISolverBase() {}

		virtual void init() = 0;
		virtual void solve(const double dt) = 0;
		virtual ~GISolverBase() {}
	};

	typedef std::shared_ptr<GISolverBase> GISolver;


	class GISolverSubstep : public GISolverBase
	{
	public:
		GISolverSubstep(GISolver& s, int nbsteps) :
			_solver(s), _steps(nbsteps)
		{}

		~GISolverSubstep() {}

		void init() { _solver->init(); }

		void solve(const double dt)
		{
			const double dta = dt / _steps;
			for (int i = 0; i < _steps; i++) { _solver->solve(dta); }
		}

	private:
		GISolver _solver;
		double _steps;
	};

	class LeapFrogSolver : public GISolverBase
	{
	public:
		LeapFrogSolver(GISolver& A, GISolver& B) : a(A), b(B){}
		~LeapFrogSolver() {}

		// Position and Velocity
		void init() { a->init(); b->init(); }

		void solve(const double dt)
		{
			const double dtd2 = 0.5 * dt;
			a->solve(dtd2);
			b->solve(dt);
			a->solve(dtd2);
		}

	private:
		GISolver a;
		GISolver b;
	};

	class ForwardEulerSolver : public GISolverBase
	{
	public:
		ForwardEulerSolver(GISolver& A, GISolver& B) : a(A), b(B){}
		~ForwardEulerSolver() {}

		void init() { a->init(); b->init(); }

		void solve(const double dt)
		{
			a->solve(dt);
			b->solve(dt);
		}

	private:
		GISolver a;
		GISolver b;
	};
	GISolver CreateGISolverSubstep(GISolver& s, int nbsteps);
	GISolver CreateForwardEulerSolver(GISolver& A, GISolver& B);
	GISolver CreateLeapFrogSolver(GISolver& A, GISolver& B);
}

#endif
