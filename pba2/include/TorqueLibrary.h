#ifndef __PBA_TORQUELIBRARY_H__
#define __PBA_TORQUELIBRARY_H__

#include "Torque.h"

namespace pba
{
	class TorqueFromForce :public TorqueBase
	{
	public:
		TorqueFromForce(Force& f):
			force(f)
		{}
		~TorqueFromForce() {}

		void compute(RigidBodyState& s, const double dt);
	
	private:
		Force force;
	};

	Torque CreateTorqueFromForce(Force& f);
}

#endif // __PBA_TORQUELIBRARY_H__