#ifndef __PBA_TORQUE_H__
#define __PBA_TORQUE_H__

#include "RigidBodyState.h"
#include "Force.h"
#include <iostream>

namespace pba
{
	class TorqueBase
	{
	public:
		TorqueBase() {};

		virtual void compute(RigidBodyState& s, const double dt) { std::cout << "calling TorqueBase::compute(RigidBodyState,double) base class virtual method\n"; }
		//virtual void compute(SoftBodyState& s, const double dt) { std::cout << "calling TorqueBase::compute(SoftBodyState,double) base class virtual method\n"; }
		virtual ~TorqueBase() {};
	};

	typedef std::shared_ptr<TorqueBase> Torque;
}
#endif // __PBA_TORQUE_H__
