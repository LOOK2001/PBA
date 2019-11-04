#include "RigidBodyState.h"

pba::RigidBodyStateData::RigidBodyStateData(const std::string& nam /*= "RBDDataNoName"*/)
{
	create_attr("r", Vector());
	create_attr("p", Vector());
	create_attr("tau", Vector());
}

pba::RigidBodyStateData::RigidBodyStateData(const RigidBodyStateData& d)
{
	create_attr("r", Vector());
	create_attr("p", Vector());
	create_attr("tau", Vector());

	center_of_mass = d.center_of_mass;
	angular_rotation = d.angular_rotation;
	linear_velocity = d.linear_velocity;
	angular_velocity = d.angular_velocity;
	center_of_mass_accel = d.center_of_mass_accel;
	angular_accel = d.angular_accel;
	angular_momentum = d.angular_momentum;

	moment_of_inertia = d.inertia_moment();
	inverse_moment_of_inertia = d.inverse_moi();
	total_mass = d.totalmass();
}

pba::RigidBodyStateData& pba::RigidBodyStateData::operator=(const RigidBodyStateData& d)
{
	if (this != &d)
	{
		create_attr("r", Vector());
		create_attr("p", Vector());
		create_attr("tau", Vector());

		center_of_mass = d.center_of_mass;
		angular_rotation = d.angular_rotation;
		linear_velocity = d.linear_velocity;
		angular_velocity = d.angular_velocity;
		center_of_mass_accel = d.center_of_mass_accel;
		angular_accel = d.angular_accel;
		angular_momentum = d.angular_momentum;

		moment_of_inertia = d.inertia_moment();
		inverse_moment_of_inertia = d.inverse_moi();
		total_mass = d.totalmass();
	}
	return *this;
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

double pba::RigidBodyStateData::total_engery() const
{
	double p1 = 0.5 * total_mass * std::pow(linear_velocity.magnitude(), 2);
	double p2 = 0.5 * angular_velocity * moment_of_inertia * angular_velocity;
	return (p1 + p2);
}

pba::Vector pba::RigidBodyStateData::vert_pos(const size_t p) const
{
	return pos(p);
}

pba::RigidBodyState pba::CreateRigidBody(const std::string& nam /*= "RigidBodyDataNoName"*/)
{
	return RigidBodyState(new RigidBodyStateData());
}

pba::RigidBodyState pba::copy(const RigidBodyState d)
{
	return RigidBodyState(new RigidBodyStateData(*d.get()));
}

