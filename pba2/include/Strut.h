#ifndef __PBA_STRUT_H__
#define __PBA_STRUT_H__

#include "Vector.h"
#include "DynamicalState.h"
#include "SoftBodyState.h"

namespace pba 
{
	class Strut
	{
	public:
		Strut(SoftEdge& e, const double k, const double f);
		~Strut();

		void Init(const SoftBodyState& s);
		void compute(SoftBodyState& s);

		const double& get_spring_constant() const { return spring_constant; }
		void set_spring_constant(const double v) { spring_constant = v; }

		const double& get_friction_coeff() const { return friction_coeff; }
		void set_friction_coeff(const double v) { friction_coeff = v; }

	private:
		SoftEdge edge;
		double spring_constant;
		double friction_coeff;
	};
}

#endif // __PBA_STRUT_H__
