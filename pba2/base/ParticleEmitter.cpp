#include "ParticleEmitter.h"

#include <random>

pba::ParticleEmitter::ParticleEmitter(const Vector& loc, const Vector& velocity, const double R, const double rms_vel)
{
	location = loc;
	mean_velocity = velocity;
	radius = R;
	velocity_rms = rms_vel;
}

void pba::ParticleEmitter::emit(Vector& p, Vector& v, Color& c)
{
	// Seed with a real random value, if available
	std::random_device r;
	std::default_random_engine e1(r());

	//std::uniform_real_distribution<float> uniform_pos(0, 5);
	std::uniform_real_distribution<float> uniform_pos(-radius, radius);
	p = Vector(uniform_pos(e1), uniform_pos(e1), uniform_pos(e1));

	//std::uniform_real_distribution<float> uniform_vel(0, 1);
	std::uniform_real_distribution<float> uniform_vel(-velocity_rms, velocity_rms);
	v = Vector(uniform_vel(e1), uniform_vel(e1), uniform_vel(e1));

	//std::uniform_real_distribution<float> uniform_color(0, 1);
	std::uniform_real_distribution<float> uniform_color(0, 1);
	c = Color(uniform_color(e1), uniform_color(e1), uniform_color(e1), 1.0f);
}
