#ifndef __PBA_SPHFORCE_H__
#define __PBA_SPHFORCE_H__

#include "Force.h"
#include "SPHState.h"

namespace pba
{
	class SPHForce : public ForceBase
	{
	public:
		SPHForce();
		~SPHForce() {}

		void compute(SPHState& s, const double dt);
		
		void compute_pressure(SPHState& s);

		void compute_viscosity(SPHState& s);

		const float& get_pressure_base() const { return pressure_base; }
		const float& get_pressure_magnitude() const { return pressure_magnitude; }
		const float& get_pressure_power() const { return pressure_power; }
		const float& get_epsilon_sph() const { return epsilon_sph; }
		const float& get_alpha_sph() const { return alpha_sph; }
		const float& get_beta_sph() const { return beta_sph; }

		void set_pressure_base(const float v) { pressure_base = v; }
		void set_pressure_magnitude(const float v) { pressure_magnitude = v; }
		void set_pressure_power(const float v) { pressure_power = v; }
		void set_epsilon_sph(const float v) { epsilon_sph = v; }
		void set_alpha_sph(const float v) { alpha_sph = v; }
		void set_beta_sph(const float v) { beta_sph = v; }

	private:
		float pressure_magnitude;
		float pressure_power;
		float pressure_base;

		float epsilon_sph;
		float alpha_sph;
		float beta_sph;

		float sound_speed(const float density) const;
	};

	Force CreateSPHForce();
}

#endif // 
