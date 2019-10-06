#ifndef __PBA_FORCE_H__
#define __PBA_FORCE_H__

#include "DynamicalState.h"
#include <iostream>

namespace pba
{
	class ForceBase
	{
	public:
		ForceBase() {};

		virtual void compute(DynamicalState& s, const double dt) { std::cout << "ForceBase::compute"; }
	};

	typedef std::shared_ptr<ForceBase> Force;
}

#endif // 
