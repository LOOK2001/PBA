#ifndef __PBA_RBDSOLVER_H__
#define __PBA_RBDSOLVER_H__

#include "RigidBodyState.h"
#include "Torque.h"
#include "TorqueLibrary.h"
#include "GISolver.h"
#include "CollisionHandler.h"

namespace pba 
{
	class AdvanceRotationAndCOM : public GISolverBase
	{
	public:
		AdvanceRotationAndCOM(RigidBodyState& pq);
		~AdvanceRotationAndCOM() {}

		void init() {}
		void solve(const double dt);

	private:
		RigidBodyState PQ;
	};

	class AdvanceAngularVelocityAndVelocity : public GISolverBase
	{
	public:
		AdvanceAngularVelocityAndVelocity(RigidBodyState& pq, Force& f);
		~AdvanceAngularVelocityAndVelocity() {}

		void init() {}
		void solve(const double dt);

	private:
		RigidBodyState PQ;
		Torque tau;

	};
	GISolver CreateAdvanceRotation(RigidBodyState& pq);
	GISolver CreateAdvanceAngularVelocity(RigidBodyState& pq, Force& f);

	class AdvanceRotationWithCollisions :public GISolverBase
	{
	public:
		AdvanceRotationWithCollisions(RigidBodyState& pq, ElasticCollisionHandler& coll);
		~AdvanceRotationWithCollisions() {}

		void init() {}
		void solve(const double dt);

	private:
		RigidBodyState PQ;
		ElasticCollisionHandler& CS;
	};
	GISolver CreateAdvanceRotation(RigidBodyState& pq, ElasticCollisionHandler& cs);
}

#endif // __PBA_RBDSOLVER_H__
