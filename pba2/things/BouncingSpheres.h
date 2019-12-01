#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
//#include "PoincareData.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "ParticleEmitter.h"
#include "PbaUtils.h"

#include <cstdlib>

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
#include <OpenGL/glu.h>  // GLU support library.
#include <GLUT/glut.h>
#else
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

#include <iostream>

using namespace std;

namespace pba {
	class BouncingSpheresThing : public PbaThingyDingy
	{
	public:
		BouncingSpheresThing(const std::string nam = "BouncingSpheresThing"):
			PbaThingyDingy(nam),
			display_pp(false),
			emit(false),
			//poincare(pba::PoincareData(300, 0, 0)),
			box(pba::makeCollisionSurface()),
			emitter(ParticleEmitter(Vector(0, 0, 0), Vector(0, 0, 0), 0.25, 1.0))
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->create_attr("pscale", 0.25f);
			//state->create_attr("pscale", 0.25f);
			state->add(2);
// 			state->set_pos(0, Vector(-2.5, 2.5, 0.0));
// 			state->set_vel(0, Vector(0.5, -0.5, 0.0));
// 			state->set_pos(1, Vector(2.5, -2.5, 0.0));
// 			state->set_vel(1, Vector(-0.5, 0.5, 0.0));
			force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
			GISolver solvera = CreateAdvancePosition(state, collisions);
			GISolver solverb = CreateAdvanceVelocity(state, force);
			//solver = CreateForwardEulerSolver(solvera,solverb);
			solver = CreateLeapFrogSolver(solvera, solverb);
			//solver = CreateRK4Solver(state,force);
			Reset();
			std::cout << name << " constructed\n";
		}
		~BouncingSpheresThing() {};

		void Init(const std::vector<std::string>& args)
		{
			CollisionSurface surf = pba::GenerateCollisionSphere();
			AddCollisionSurface(surf);
			collisions.set_collision_surface(surf);

			toggleAnimate();
		}

		// Callback functions
		void Display()
		{
			pba::Display(box);

			for (size_t i = 0; i < state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glTranslatef(v.X(), v.Y(), v.Z());
				glutSolidSphere(state->get_float_attr("pscale", i), 20, 10);
				glTranslatef(-v.X(), -v.Y(), -v.Z());
			}
		}

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'p') { display_pp = !display_pp; }
			if (key == 'e') { emit = !emit; }
			if (key == 's')
			{
				collisions.do_self_collisions = !collisions.do_self_collisions;
				if (collisions.do_self_collisions) { std::cout << "Sphere-Sphere collisions ON\n"; }
				if (!collisions.do_self_collisions) { std::cout << "Sphere-Sphere collisions OFF\n"; }
			}
			if (key == 'g')
			{
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
				f->set_gravity_constant(f->get_gravity_constant() / 1.1);
			}
			if (key == 'G')
			{
				std::shared_ptr<SimpleGravityForce> f = dynamic_pointer_cast<SimpleGravityForce>(force);
				f->set_gravity_constant(f->get_gravity_constant() * 1.1);
			}
		};

		void solve()
		{
			if (emit)
			{
				int nbincrease = 10;
				state->add(nbincrease);
				Vector P, V;
				Color C;
				std::cout << "Emit Points " << state->nb() << std::endl;
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					emitter.emit(P, V, C);
					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);
				}
			}
			solver->solve(dt);
		};

		void Reset()
		{
			// Distribute particles with random positions
			Vector P, V;
			Color C;
			for (size_t i = 0; i < state->nb(); i++)
			{
				emitter.emit(P, V, C);
				state->set_pos(i, P);
				state->set_vel(i, V);
				state->set_ci(i, C);
			}
		};

		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "p            toggle between normal display and Poincare plot\n";
			cout << "SPACEBAR     start/stop animation\n";
			cout << "t/T          reduce/increase animation time step\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "e            toggle sphere emission on/off\n";
			cout << "s            toggle sphere-sphere collisions on/off\n";
		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			collisions.set_collision_surface(box);
		}


	private:
		bool display_pp;
		bool emit;

		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;
		//pba::PoincareData poincare;

		std::vector<CollisionTriangle> triangles;
		pba::CollisionSurface box;
		pba::ElasticSphereCollisionHandler collisions;
		pba::ParticleEmitter emitter;
	};

	pba::PbaThing BouncingSpheres() { return PbaThing(new BouncingSpheresThing()); }
}