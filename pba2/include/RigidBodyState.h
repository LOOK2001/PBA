#ifndef __PBA_RIGIDBODYSTATE_H__
#define __PBA_RIGIDBODYSTATE_H__

#include "DynamicalState.h"
#include "Matrix.h"
#include "LinearAlgebra.h"

namespace pba
{
	class RigidBodyStateData : public DynamicalStateData
	{
	public:
		RigidBodyStateData(const std::string& nam = "RBDDataNoName");
		RigidBodyStateData(const RigidBodyStateData& d);
		~RigidBodyStateData();

		RigidBodyStateData& operator= (const RigidBodyStateData& d);

		void compute_RBD_data();
		void compute_M();
		void recompute_MOI();

		const Matrix& inertial_moment() const { return moment_of_inertia; }
		const Matrix& inverse_moi() const { return inverse_moment_of_inertia; }
		const float totalmass() const { return total_mass; }

		const Matrix& raw_moi() const { return moment_of_inertia; }

		double total_engery() const;

		Vector vert_pos(const size_t p) const;

		Vector center_of_mass;
		Matrix angular_rotation;
		Vector linear_velocity;
		Vector angular_velocity;
		Vector center_of_mass_accel;
		Vector angular_accel;
		Vector angular_momentum;

	private:
		Matrix moment_of_inertia;
		Matrix inverse_moment_of_inertia;
		float total_mass;
	};

	typedef std::shared_ptr<RigidBodyStateData> RigidBodyState;
	RigidBodyState CreateRigidBody(const std::string& nam = "RigidBodyDataNoName");
	RigidBodyState copy(const RigidBodyState d);
}

#endif // __PBA_RIGIDBODYSTATE_H__
