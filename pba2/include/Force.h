#ifndef __PBA_FORCE_H__
#define __PBA_FORCE_H__

#include "DynamicalState.h"
#include "RigidBodyState.h"
#include "SoftBodyState.h"
#include "SPHState.h"
#include <iostream>

namespace pba
{
	class ForceBase
	{
	public:
		ForceBase() {};

		virtual void compute(DynamicalState& s, const double dt) { std::cout << "calling ForceBase::compute(DynamicalState, double) base class virtual method\n"; }
		virtual void compute(RigidBodyState& s, const double dt) { std::cout << "calling ForceBase::compute(RigidBodyState, double) base class virtual method\n"; }
		virtual void compute(SoftBodyState& s, const double dt) { std::cout << "calling ForceBase::compute(SoftBodyState, double) base class virtual method\n"; }
		virtual void compute(SPHState& s, const double dt) { std::cout << "calling ForceBase::compute(SPHState, double) base class virtual method\n"; }
		virtual ~ForceBase() {};
	};

	typedef std::shared_ptr<ForceBase> Force;
}

#endif // 
