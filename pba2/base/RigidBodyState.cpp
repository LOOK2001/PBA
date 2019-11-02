#include "RigidBodyState.h"

pba::RigidBodyStateData::RigidBodyStateData(const std::string& nam /*= "RBDDataNoName"*/)
{
	create_attr("r", Vector());
	create_attr("p", Vector());
	create_attr("tau", Vector());
}

void pba::RigidBodyStateData::compute_RBD_data()
{
	for (int i = 0; i < nb(); i++) {
		Vector homePos = pos(i);
		set_attr("p", i, homePos);
	}

	angular_rotation = unitMatrix();

	total_mass = 0;
	Vector _p, _v;
	for (int i = 0; i < nb(); i++)
	{
		total_mass += mass(i);
		_p += (mass(i) * pos(i));
		_v += (mass(i) * vel(i));
	}

	center_of_mass = _p / total_mass;
	linear_velocity = Vector();
	angular_velocity = Vector();

	for (int i = 0; i < nb(); i++)
	{
		Vector r = pos(i) - center_of_mass;
		set_attr("r", i, r);
	}

	recompute_MOI();

	angular_momentum = moment_of_inertia * angular_velocity;
}

void pba::RigidBodyStateData::compute_M()
{
// 	auto kD = [] (int i, int j){
// 		return (i == j) ? true : false;
// 	};
// 
// 	for (int i = 0; i < 3; i++){
// 		for (int j = 0; j < 3; j++){
// 			for (int k = 0; k < nb(); k++) {
// 				const Vector& r = get_vector_attr("r", k);
// 				const double r_mag = r.magnitude();
// 				moment_of_inertia[i][j] += mass(k)*(r_mag* r_mag * kD(i, j) - r[i]*r[j]);
// 			}
// 		}
// 	}
}

void pba::RigidBodyStateData::recompute_MOI()
{
	Matrix unitM = pba::unitMatrix();
	for (int i = 0; i < nb(); i++) {
		const Vector& r = get_vector_attr("r", i);
		Matrix m;
		outer_product(r, r, m);
		const double r_mag = r.magnitude();
		moment_of_inertia += mass(i) * (r_mag * r_mag * unitM - m);
	}

	inverse_moment_of_inertia = inverse(moment_of_inertia);
}

