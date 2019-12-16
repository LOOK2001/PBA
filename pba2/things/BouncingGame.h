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

#define PBAGMAE

using namespace std;

namespace pba {
	class BouncingGameThing : public PbaThingyDingy
	{
	public:
		BouncingGameThing(const std::string nam = "BouncingSpheresThing") :
			PbaThingyDingy(nam),
			display_pp(false),
			emit(false),
			fire(false),
			//poincare(pba::PoincareData(300, 0, 0)),
			box(pba::makeCollisionSurface()),
			emitter(ParticleEmitter(Vector(0, 0, 0), Vector(0, 0, 0), 0.25, 1.0))
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->create_attr("pscale", 0.25f);
			state->create_attr("life", -1.0f);

			bulletState = CreateDynamicalState(name + "DynamicalData");
			bulletState->create_attr("pscale", 0.1f);

			state->add(1);
			force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
			GISolver solvera = CreateAdvancePosition(state, collisions);
			GISolver solverb = CreateAdvanceVelocity(state, force);
			solver = CreateLeapFrogSolver(solvera, solverb);

// 			GISolver bulleta = CreateAdvancePosition(bulletState, collisions);
// 			GISolver bulletb = CreateAdvanceVelocity(bulletState, force);
// 			bulletSolver = CreateLeapFrogSolver(bulleta, bulletb);

			Reset();
			std::cout << name << " constructed\n";
		}
		~BouncingGameThing() {};

		void Init(const std::vector<std::string>& args)
		{
			CollisionSurface surf = pba::GenerateCollisionPlane(5.0f);
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

// 			for (size_t i = 0; i < bulletState->nb(); i++)
// 			{
// 				const Color& ci = bulletState->ci(i);
// 				const pba::Vector& v = bulletState->pos(i);
// 				glColor3f(ci.red(), ci.green(), ci.blue());
// 				glTranslatef(v.X(), v.Y(), v.Z());
// 				glutSolidSphere(bulletState->get_float_attr("pscale", i), 20, 10);
// 				glTranslatef(-v.X(), -v.Y(), -v.Z());
// 			}
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
			if (key == 'j')
			{
				fire = !fire;
				std::cout << "fire" << std::endl;
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
			if (fire)
			{
				// fire
				int nbincrease = 1;
				//bulletState->add(nbincrease);
				state->add(nbincrease);
				Vector P, V;
				Color C;
				std::cout << "Emit Points " << state->nb() << std::endl;
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					emitter.emit(P, V, C);
					state->set_pos(i, Vector(0.0, 0.0, -2.0));
					state->set_vel(i, Vector(0.0, 0.0, 2.0));
					state->set_ci(i, C);
					state->set_attr("pscale", i, 0.08f);
					state->set_attr("life", i, 0.0f);
				}
// 				std::cout << "Emit Points " << bulletState->nb() << std::endl;
// 				for (size_t i = bulletState->nb() - nbincrease; i < bulletState->nb(); i++)
// 				{
// 					emitter.emit(P, V, C);
// 					bulletState->set_pos(i, Vector(0.0, 0.0, 0.0));
// 					bulletState->set_vel(i, Vector(0.0, 0.0, 0.5));
// 					bulletState->set_ci(i, C);
// 				}
				fire = !fire;
			}
			solver->solve(dt);

			OnUpdate(dt);
			//bulletSolver->solve(dt);
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
		bool fire;

		pba::DynamicalState state;
		pba::DynamicalState bulletState;
		pba::Force force;
		pba::GISolver solver;
		pba::GISolver bulletSolver;
		//pba::PoincareData poincare;

		std::vector<CollisionTriangle> triangles;
		pba::CollisionSurface box;
		pba::ElasticSphereCollisionHandler collisions;
		pba::ParticleEmitter emitter;

		void OnUpdate(double dt)
		{
			for (size_t i = 0; i < state->nb(); i++)
			{
				float life = state->get_float_attr("life", i);
				if (life < 0.0f)
					continue;

				life += dt;
				if (life >= 1.0)
				{
					//state->erase(i);
					state->set_attr("life", i, life);
				}
				else
					state->set_attr("life", i, life);
				
			}
		}
	};

	pba::PbaThing BouncingGame() { return PbaThing(new BouncingGameThing()); }
}