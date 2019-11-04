#include "CollisionTriangle.h"

pba::CollisionTriangleRaw::CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2)
{
	P0 = p0; P1 = p1; P2 = p2;
	visible = true;

	e1 = P1 - P0; e2 = P2 - p0;
	normal = (e1 ^ e2);
	normal.normalize();
}

bool pba::CollisionTriangleRaw::hit(const Vector& P, const Vector& V, const double tmax, double& t)
{
	// Detect a collision has happened
	double res1 = (P - P0) * normal;
	double res2 = ((P - (V * tmax)) - P0) * normal;
	if ((res1 * res2) > 0)
		return false;

	// Compute where and when collision takes place
	t = (normal * (P - P0)) / (normal * V);
	Vector xc = P - (V * t);

	// not a collision
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

bool pba::CollisionTriangleRaw::hit(const RigidBodyState& s, const size_t i, const double tmax, double& t)
{
	Matrix u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * tmax) * s->angular_rotation;
	Vector x0 = s->center_of_mass - s->linear_velocity * tmax + u * s->get_vector_attr("p", i);
	Vector x1 = s->pos(i);

	// Detect a collision has happened
	double f0 = (x0 - P0) * normal;
	double f1 = (x1 - P0) * normal;
	if ((f0 * f1) > 0)
		return false;

	// Compute where and when collision takes place
	double t0 = 0, t1 = tmax;
	Vector xc;
	double fc;
	while (true)
	{
		t = 0.5 * (t0 + t1);

		u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * t) * s->angular_rotation;
		xc = s->center_of_mass - s->linear_velocity * t + u * s->get_vector_attr("p", i);
		fc = (xc - P0) * normal;

		if (fc == 0){
			t = (t0 + t1) * 0.5;
			break;
		}

		if (f0 * fc < 0){
			f1 = fc;
			t0 = t;
		}
		else if (f0 * fc > 0){
			f0 = fc;
			t1 = t;
		}

		if (abs(abs(t0 - t1) / tmax) < 0.0001)
			break;
	}

	u = rotation(s->angular_velocity, s->angular_velocity.magnitude() * t)* s->angular_rotation;
	xc = s->center_of_mass - s->linear_velocity*t + u * s->get_vector_attr("p", i);
	
	// not a collision
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

bool pba::CollisionTriangleRaw::is_in_triangle(const Vector& X)
{
	double u = ((e2 ^ e1) * (e2 ^ (X - P0))) / (pow(((e2 ^ e1).magnitude()), 2));
	double v = ((e1 ^ e2) * (e1 ^ (X - P0))) / (pow(((e1 ^ e2).magnitude()), 2));

	if ((0 <= u) && (u <= 1) &&
		((0 <= v) && (v <= 1)) &&
		((0 <= (v + u)) && ((v + u) <= 1)))
		return true;

	return false;
}

pba::CollisionTriangle pba::makeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2)
{
	return CollisionTriangle(new CollisionTriangleRaw(p0, p1, p2));
}
