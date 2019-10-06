#ifndef __PBA_EXPLICITDYNAMICS_H__
#define __PBA_EXPLICITDYNAMICS_H__

#include "DynamicalState.h"
#include "Force.h"
#include "Constraint.h"
#include "GISolver.h"
#include "CollisionHandler.h"

namespace pba
{
	// Solvers with no collisions or constraints
	class AdvancePosition : public GISolverBase
	{
	public:
		AdvancePosition(DynamicalState& pq);
		~AdvancePosition() {}

		void init();
		void solve(const double dt);

	private:
		DynamicalState PQ;
	};

	class AdvancedVelocity : public GISolverBase
	{
	public:
		AdvancedVelocity(DynamicalState& pq, Force& f);
		~AdvancedVelocity() {}

		void init();
		void solve(const double dt);

	private:
		DynamicalState PQ;
		Force force;
	};

	GISolver CreateAdvancePosition(DynamicalState& pq);
	GISolver CreateAdvanceVelocity(DynamicalState& pq, Force& f);

	// Solvers that involve collisions
	class AdvancePositionWithCollisions : public GISolverBase
	{
	public:
		AdvancePositionWithCollisions(DynamicalState& pq, CollisionHandler& coll);
		~AdvancePositionWithCollisions() {}

		void init();
		void solve(const double dt);

	private:
		DynamicalState PQ;
		CollisionHandler* CS;
	};
	GISolver CreateAdvancePosition(DynamicalState& pq, CollisionHandler& cs);

	// Solvers that involve constraints
	class AdvanceVelocityWithConstraint : public GISolverBase
	{
	public:
		AdvanceVelocityWithConstraint(DynamicalState& pq, Force& f, MultiConstraint& c);
		~AdvanceVelocityWithConstraint() {}

		void init() {};
		void solve(const double dt);
	
	private:
		DynamicalState PQ;
		Force force;
		MultiConstraint C;
	};
	GISolver CreateAdvanceVelocity(DynamicalState& pq, Force& f, MultiConstraint& c);

	class AdvancePositionWithConstraint : public GISolverBase
	{
	public:
		AdvancePositionWithConstraint(DynamicalState& pq, MultiConstraint& c);
		~AdvancePositionWithConstraint() {}

		void init() {};
		void solve(const double dt);

	private:
		DynamicalState PQ;
		MultiConstraint C;
		double tolerance;
		int maxloops;

	};
	GISolver CreateAdvancePosition(DynamicalState& pq, MultiConstraint& cs);
}

#endif
