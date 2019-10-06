#ifndef __PBA_AABB_H__
#define __PBA_AABB_H__

#include "Vector.h"
#include "Matrix.h"

namespace pba {
	class AABB
	{
	public:
		AABB(const Vector& lc, const Vector& uc);
		~AABB(){};

		const bool intersects(const AABB& aabb) const;
		AABB Intersection(const AABB& aabb) const;
		AABB Union(const AABB& aabb) const;
		void split(const int component, AABB& aabb1, AABB& aabb2) const;
		const double volume() const;
		const bool isInside(const Vector& P) const;
		const double intersect(const Vector& start, const Vector& direction) const;
		const double intersect(const Vector& P, const Vector& V, const Vector& W, const Matrix& rot, const Vector& com) const;
		const Vector& LLC() const { return llc; }
		const Vector& URC() const {return urc;}

	private:
		Vector llc;
		Vector urc;
	};
}

#endif
