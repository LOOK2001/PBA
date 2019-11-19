#ifndef __PBA_COLLISIONSURFACE_H__
#define __PBA_COLLISIONSURFACE_H__

#include "CollisionTriangle.h"
#include "RigidBodyState.h"
#include "SoftBodyState.h"

#include <vector>
#include <memory>

namespace pba {

	struct CollisionData
	{
		double t;
		CollisionTriangle tri;
		bool status;
		bool hit_tri;
		bool hit_sph;
		size_t hit_index;
	};

	class CollisionSurfaceRaw
	{
	public:
		CollisionSurfaceRaw();
		~CollisionSurfaceRaw() {}

		void addTriangle(const CollisionTriangle& t);
		//void addSphere(const CollisionSphere& t);
		void clear() { tri_elements.clear(); }
		size_t triangle_size() const { return tri_elements.size(); }

		bool hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const;

		bool hit(const RigidBodyState& s, const size_t i, const double tmax, CollisionData& t) const;

		bool hit(const SoftBodyState& s, const size_t i, const double tmax, CollisionData& t) const;

		//bool hit(const Vector& P, const Vector& V, const double R, const double tmax, CollisionData& t) const;

		CollisionTriangle& get_triangle(size_t i) { return tri_elements[i]; }
		const CollisionTriangle& get_triangle(size_t i) const { return tri_elements[i]; }

		void toggle_visible() { visible = !visible; }
		bool is_visible() const { return visible; }
		void toggle_wireframe() { wireframe = !wireframe; }
		bool use_wireframe() const { return wireframe; }

		void set_coeff_restitution(const double v) { coeff_of_restitution = v; }
		const double& coeff_restitution() const { return coeff_of_restitution; }

		void set_coeff_sticky(const double v) { coeff_of_sticky = v; }
		const double& coeff_sticky() const { return coeff_of_sticky; }

		void set_up_aabb();

		const AABB& aabb() const { return aa_bb; }

		void decay();

	private:
		bool visible;
		bool wireframe;

		double coeff_of_restitution;
		double coeff_of_sticky;

		std::vector<CollisionTriangle> tri_elements;
		//std::vector<CollisionSphere> sphere_elements;

		AABB aa_bb;
	};

	typedef std::shared_ptr<CollisionSurfaceRaw> CollisionSurface;

	CollisionSurface makeCollisionSurface();
}

#endif
