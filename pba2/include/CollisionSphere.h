
#ifndef __PBA_COLLISIONSPHERE_H__
#define __PBA_COLLISIONSPHERE_H__

#include "Vector.h"
#include "Color.h"
#include "AABB.h"
#include <memory>

namespace pba {

	class CollisionSphereRaw
	{
	public:
		CollisionSphereRaw(const Vector& p0, const double& R);
		~CollisionSphereRaw() {}

		// collision with a particle
		bool hit(const Vector& P, const Vector& V, const double tmax, double& t);

		bool hit(const Vector& P, const Vector& V, const double R, const double tmax, double& t);

		const Vector& center() const { return cen; }
		const double& radius() const { return rad; }

		void set_visible() { visible = true; }
		void set_invisible() { visible = false; }
		bool visibility() const { return visible; }

		void set_color(const Color& c) { color = c; }
		const Color get_color() const { return color + hitcolor; }

		void decay();

		const AABB& aabb() const { return aa_bb; }

	private:
		Vector cen;
		double rad;

		Color color;
		Color hitcolor;
		float decayrate;
		bool visible;

		AABB aa_bb;
	};
	typedef std::shared_ptr<CollisionSphereRaw> CollisionSphere;

	CollisionSphere makeCollisionSphere(const Vector& p0, const double& R);
}
#endif