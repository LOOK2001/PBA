#include "Vector.h"
#include "AABB.h"
#include "CollisionSurface.h"
//#include "RigidBodyState.h"


namespace pba {
	class TraceTree {
	public:
		TraceTree(const Vector& llc, const Vector& urc, const int lvl, const int maxlvl, const int minobj);
		~TraceTree();

		void Divide();

		void addObject(CollisionSurface& s);
		void addObject(CollisionTriangle& s);

		bool hit(const Vector& P, const Vector& V, const double tmax, CollisionData& t) const;

		bool hit(const RigidBodyState& s, const size_t i, const double tmax, CollisionData& t) const;

		//bool hit()

		const size_t nbObject() const { return object_list.size(); }

	private:
		AABB aabb;
		TraceTree* node1;
		TraceTree* node2;

		int level;
		int max_levels;
		int min_objects;

		std::vector<CollisionTriangle> object_list;
	};

}