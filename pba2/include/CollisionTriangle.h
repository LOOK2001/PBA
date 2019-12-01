#ifndef __PBA_COLLISIONTRIANGLE_H__
#define __PBA_COLLISIONTRIANGLE_H__

#include "Vector.h"
#include "Color.h"
#include "RigidBodyState.h"
#include "SoftBodyState.h"
#include "AABB.h"
#include <memory>

namespace pba {
	class CollisionTriangleRaw
	{
	public:
		CollisionTriangleRaw(const Vector& p0, const Vector& p1, const Vector& p2);
		~CollisionTriangleRaw() {}

		bool hit(const Vector& P, const Vector& V, const double tmax, double& t);

		bool hit(const RigidBodyState& s, const size_t i, const double tmax, double& t);

		bool hit(const SoftBodyState& s, const size_t i, const double tmax, double& t);

		bool hit(const Vector& P, const Vector& V, const double R, const double tmax, double& t);

		const Vector& N() const { return normal; }
		const Vector& vertex(int i) const
		{
			if (i == 2) { return P2; }
			else if (i == 1) { return P1; }
			return P0;
		}

		void set_visible() { visible = true; }
		void set_invisible() { visible = false; }
		bool visibility() const { return visible; }

		void set_color(const Color& c) { color = c; }
		const Color get_color() const { return color + hitcolor; }

		void set_hit_color(const Color& c) { hitcolor = c; }
		void set_lit_color(const Color& c) { litcolor = c; }

		void decay();

		void set_orientation(const Vector& P);

	private:
		Vector P0;
		Vector P1;
		Vector P2;
		Vector e1;
		Vector e2;
		Vector normal;
		double det;

		Color color;
		Color hitcolor;
		Color litcolor;
		float decayrate;
		bool visible;
		
		//AABB aa_bb;

		bool is_in_triangle(const Vector& X);
	};

	typedef std::shared_ptr<CollisionTriangleRaw> CollisionTriangle;
	CollisionTriangle makeCollisionTriangle(const Vector& p0, const Vector& p1, const Vector& p2);
}

#endif
