#ifndef __PBA_FORCELIBRARY_H__
#define __PBA_FORCELIBRARY_H__

#include "Force.h"

namespace pba
{
	class SimpleGravityForce : public ForceBase
	{
	public:
		SimpleGravityForce(const pba::Vector& g) : G(g) {}
		~SimpleGravityForce(){}

		void compute(pba::DynamicalState& pq, const double dt);

		void set_gravity_constant(const double v) { G = G.unitvector() * v; }
		const double get_gravity_constant() const { return G.magnitude(); }

	private:
		pba::Vector G;
	};
	pba::Force CreateSimpleGravityForce(const pba::Vector& G);


	class AccumulatingForce : public ForceBase
	{
	public:
		AccumulatingForce() {}
		~AccumulatingForce() {}

		void compute(pba::DynamicalState& pq, const double dt);

		void add(Force& f);

	private:
		std::vector<Force> forces;
	};
	Force CreateAccumulatingForce();


	class AccumulatingBoidForce : public pba::ForceBase
	{
	public:
		AccumulatingBoidForce(const double a, const double v, const double c, const double Max, const double rng, const double rng_ramp = 1.0, int leadBoid = 0) :
			A(a),
			V(v),
			C(c),
			amax(Max),
			range(rng),
			range_ramp(rng_ramp),
			fov(152.0),
			dfov(10.0)
		{cosfov = std::cos(fov * 3.14159265 / 360.0);
		cosfovshell = std::cos(dfov * 3.14159265 / 360.0);}
		~AccumulatingBoidForce() {};

		void compute(pba::DynamicalState& pq, const double dt);

		void set_avoidance(const double v) { A = v; };
		const double get_avoidance() const { return A; };

		void set_matching(const double v) { V = v; };
		const double get_matching() const { return V; };

		void set_centering(const double v) { C = v; };
		const double get_centering() const { return C; };

		void set_max(const double v) { amax = v; };
		const double get_max() const { return amax; };

		void set_range(const double v) { range = v; };
		const double get_range() const { return range; };

		void set_range_ramp(const double v) { range_ramp = v; };
		const double get_range_ramp() const { return range_ramp; };

		void set_fov(const double v) { fov = v; if (fov > 360.0) { fov = 360.0; } cosfov = std::cos(v * 3.14159265 / 360.0); set_fov_shell(dfov); };
		const double get_fov() const { return fov; };

		void set_fov_shell(const double v) { dfov = v; if (dfov > fov) { dfov = fov; } cosfovshell = cosfov = std::cos(v * 3.14159265 / 360.0); }
		const double get_fov_shell() const { return dfov; };

		void set_lead_boid_id(const int _id) { leadBoidID = _id; }
		const int get_lead_boid_id() { return leadBoidID; }

	private:
		double A, V, C;
		double amax;
		double range, range_ramp;
		double fov;
		double dfov;
		double cosfov;
		double cosfovshell;
		int leadBoidID;
	};
	pba::Force CreateAccumulatingBoidForce(const double A, const double V, const double C, const double Max, const double range, const double range_ramp);


	class AccumulatingRandomBoidForce : public pba::ForceBase
	{
	public:
		AccumulatingRandomBoidForce() {};
		~AccumulatingRandomBoidForce() {};

		void compute(pba::DynamicalState& pq, const double dt);

	private:
	};
	pba::Force CreateAccumulatingRandomBoidForce();
}

#endif
