#ifndef __PBA_PARTICLEEMITTER_H__
#define __PBA_PARTICLEEMITTER_H__

#include "Vector.h"
#include "Color.h"

namespace pba {
	class ParticleEmitter
	{
	public:
		ParticleEmitter(const Vector& loc, const Vector& velocity, const double R, const double rms_vel);
		~ParticleEmitter() {}

		virtual void emit(Vector& p, Vector& v, Color& c);

	private:
		Vector location;
		Vector mean_velocity;
		double radius;
		double velocity_rms;
	};
}
#endif
