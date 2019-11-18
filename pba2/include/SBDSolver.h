#ifndef __PBA_SBDSOLVER_H__
#define __PBA_SBDSOLVER_H__


#include "SoftBodyState.h"
#include "Force.h"
#include "GISolver.h"
#include "CollisionHandler.h"

namespace pba
{
	class AdvanceSoftBodyPosition : public GISolverBase
	{
	public:
		AdvanceSoftBodyPosition(SoftBodyState& pq) :
			PQ(pq) 
		{}
		~AdvanceSoftBodyPosition() {}

		void init() {};
		void solve(const double dt);

	private:
		SoftBodyState PQ;
		GISolver rbdsolver;
	};

	class AdvanceSoftBodyVelocity : public GISolverBase
	{
	public:
		AdvanceSoftBodyVelocity(SoftBodyState& pq, Force& f) :
			PQ(pq), force(f) 
		{}
		~AdvanceSoftBodyVelocity() {}

		void init() {};
		void solve(const double dt);

	private:
		SoftBodyState PQ;
		Force force;
	};
	GISolver CreateAdvanceRotation(SoftBodyState& pq);
	GISolver CreateAdvanceAngularVelocity(SoftBodyState& pq, Force& f);

	class AdvanceSoftBodyPositionWithCollisions : public GISolverBase
	{
	public:
		AdvanceSoftBodyPositionWithCollisions(SoftBodyState& pq, ElasticSBDCollisionHandler& coll) :
			PQ(pq), CS(coll) {}
		~AdvanceSoftBodyPositionWithCollisions() {}

		void init() {};
		void solve(const double dt);

	private:
		SoftBodyState PQ;
		GISolver rbdsolver;
		ElasticSBDCollisionHandler& CS;
	};

	GISolver CreateAdvanceRotation(SoftBodyState& pq, ElasticSBDCollisionHandler& cs);
}

#endif