#ifndef __PBA_SPHSOLVER_H__
#define __PBA_SPHSOLVER_H__

#include "SPHState.h"
#include "Force.h"
#include "GISolver.h"
#include "CollisionHandler.h"

namespace pba
{
	class AdvanceSPHPosition : public GISolverBase
	{
	public:
		AdvanceSPHPosition(SPHState& pq);
		~AdvanceSPHPosition() {}

		void init();
		void solve(const double dt);

	private:
		SPHState PQ;
		GISolver rbdsolver;
	};
	GISolver CreateAdvancePosition(SPHState& pq);


	class AdvanceSPHVelocity : public GISolverBase
	{
	public:
		AdvanceSPHVelocity(SPHState& pq, Force& f, float vclamp, float aclamp);
		~AdvanceSPHVelocity() {}

		void init() {};
		void solve(const double dt);

		const float get_velocity_clamp() const { return velocity_clamp; }
		void set_velocity_clamp(const float& v) { velocity_clamp = v; }

		const float get_acceleration_clamp() const { return acceleration_clamp; }
		void set_acceleration_clamp(const float& v) { acceleration_clamp = v; }

	private:
		SPHState PQ;
		Force force;
		float velocity_clamp;
		float acceleration_clamp;
	};
	GISolver CreateAdvanceVelocity(SPHState& pq, Force& f, float vel_clamp, float accel_clamp);


	class AdvanceSPHPositionWithCollisions : public GISolverBase
	{
	public:
		AdvanceSPHPositionWithCollisions(SPHState& pq, ElasticCollisionHandler& coll);
		~AdvanceSPHPositionWithCollisions() {}

		void init() {};
		void solve(const double dt);

	private:
		SPHState PQ;
		GISolver rbdsolver;
		ElasticCollisionHandler* CS;
	};
	GISolver CreateAdvancePosition(SPHState& pq, ElasticCollisionHandler& cs);
}

#endif // __PBA_SPHSOLVER_H__
