#ifndef __PBA_COLLISIONHANDLER_H__
#define __PBA_COLLISIONHANDLER_H__

#include "CollisionSurface.h"
#include "DynamicalState.h"
//#include "RigidBodyState.h"
//#include "SoftBodyState.h"
#include "TraceTree.h"
#include <iostream>

namespace pba
{
	// Base class for collision handling
	class CollisionHandler
	{
	public:
		CollisionHandler() {}
		virtual ~CollisionHandler() {}

		virtual void handle_collisions(const double dt, DynamicalState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };
		//virtual void handle_collisions(const double dt, RigidBodyState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };
		//virtual void handle_collisions(const double dt, SoftBodyState& s) { std::cout << "CollisionHandler::handle_collisions(double,DynamicalState) called\n"; };

		void set_collision_surface(CollisionSurface& c);

	protected:
		CollisionSurface surf;
		TraceTree* tree;
		bool usetree;
	};

	class ElasticCollisionHandler : public CollisionHandler
	{
	public:
		ElasticCollisionHandler() {}
		~ElasticCollisionHandler() {}
		void handle_collisions(const double dt, DynamicalState& S);
	};
}

#endif
