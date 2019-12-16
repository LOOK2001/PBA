#include "CollisionTriangle.h"

pba::CollisionTriangleRaw::CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2)
	:collType(WALL)
{
	P0 = p0; P1 = p1; P2 = p2;
	visible = true;

	e1 = P1 - P0; e2 = P2 - p0;
	normal = (e1 ^ e2);
	normal.normalize();

	decayrate = 0.95;
}

bool pba::CollisionTriangleRaw::hit(const Vector& P, const Vector& V, const double tmax, double& t)
{
	// Detect a collision has happened
	double res1 = (P - P0) * normal;
	double res2 = ((P - (V * tmax)) - P0) * normal;
	if (res1 == 0.0)
		return false;
	if ((res1 * res2) > 0)
		return false;

	// Compute where and when collision takes place
	t = (normal * (P - P0)) / (normal * V);
	Vector xc = P - (V * t);

	// return false;
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

bool pba::CollisionTriangleRaw::hit(const Vector& P, const Vector& V, const double R, const double tmax, double& t)
{
	// Detect a collision has happened
	double res = abs((P - P0) * normal);
	if (res >= R)
		return false;

	// Compute where and when collision takes place
	double tmpT1 = ((P - P0) * normal + R) / (V * normal);
	double tmpT2 = ((P - P0) * normal - R) / (V * normal);

	if ((tmpT1 * tmax) > 0 && (abs(tmpT1) < abs(tmax)))
		t = tmpT1;
	else if ((tmpT2 * tmax) > 0 && (abs(tmpT2) < abs(tmax)))
		t = tmpT2;
	else
		return false;

	Vector xc = P - (V * t);

	if (is_in_triangle(xc + R * normal) || is_in_triangle(xc - R * normal))
		return true;
	else {
// 		Vector e[3];
// 		e[0] = P1 - P0;
// 		e[1] = P2 - P1;
// 		e[2] = P2 - P0;
// 		
// 		Vector ver[3];
// 		ver[0] = P0;
// 		ver[1] = P1;
// 		ver[2] = P2;
// 		for (size_t i= 0; i < 3; i++)
// 		{
// 			Vector p0 = P0[i];
// 			Vector edge = e[i];
// 			Vector vv = V - edge * (edge * V) / pow((edge.magnitude()), 2);
// 			Vector sv = (P - p0) - (edge * (edge * (P - p0))) / pow((edge.magnitude()), 2);
// 			double p1 = sqrt(pow((vv * sv), 2) + pow((vv).magnitude(), 2) * (pow((sv).magnitude(), 2) - (R * R))) / (pow((sv).magnitude(), 2));
// 			double _t = 0;
// 			double t1 = (-vv * sv) / pow((vv).magnitude(), 2) + p1;
// 			double t2 = (-vv * sv) / pow((vv).magnitude(), 2) - p1;
// 
// 			if ((t1 * tmax) > 0 && (abs(t1) < abs(tmax)))
// 				_t = t1;
// 			else if ((t2 * tmax) > 0 && (abs(t2) < abs(tmax)))
// 				_t = t2;
// 			else
// 				continue;
// 
// 			double a = edge * (P - V * _t - p0) / (pow(edge.magnitude(), 2));
// 			if (!(a >= 0 && a <= 1))
// 				continue;
// 			if (abs(t) > _t)
// 			{
// 				t = _t;
// 				return true;
// 			}
// 		}
		return false;
	}
}

bool pba::CollisionTriangleRaw::hit(const SoftBodyState& s, const size_t i, const double tmax, double& t)
{
	const Vector& P = s->pos(i);
	const Vector& V = s->vel(i);

	// Detect a collision has happened
	double res1 = (P - P0) * normal;
	double res2 = ((P - (V * tmax)) - P0) * normal;
	if (res1 == 0.0)
		return false;
	if ((res1 * res2) > 0)
		return false;

	// Compute where and when collision takes place
	t = (normal * (P - P0)) / (normal * V);
	Vector xc = P - (V * t);

	// return false;
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

bool pba::CollisionTriangleRaw::hit(const RigidBodyState& s, const size_t i, const double tmax, double& t)
{
	const Vector& w = s->angular_velocity;
	Matrix u = rotation(w, s->angular_velocity.magnitude() * tmax) * s->angular_rotation;
	Vector x0 = s->center_of_mass - s->linear_velocity * tmax + u * s->get_vector_attr("p", i);
	const Vector& x1 = s->pos(i);

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

		u = rotation(s->angular_velocity.unitvector(), s->angular_velocity.magnitude() * t) * s->angular_rotation;
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

	u = rotation(s->angular_velocity.unitvector(), s->angular_velocity.magnitude() * t)* s->angular_rotation;
	xc = s->center_of_mass - s->linear_velocity*t + u * s->get_vector_attr("p", i);
	
	// not a collision
	if ((t * tmax < 0) || (((tmax - t) / tmax) < 1e-6))
		return false;

	return is_in_triangle(xc);
}

void pba::CollisionTriangleRaw::decay()
{
	if (hitcolor[0] == 0.0)
		return;

	hitcolor *= decayrate;
	if (hitcolor[0] < 0.01)
		hitcolor = Color(0.0, 0.0, 0.0, 0.0);
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
