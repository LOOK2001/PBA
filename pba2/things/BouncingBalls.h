#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
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
	class BouncingBallsThing : public PbaThingyDingy
	{
	public:
		BouncingBallsThing(const std::string nam = "BouncingBallsThing") :
			PbaThingyDingy(name), emit(false), box (pba::makeCollisionSurface()),
			emitter(ParticleEmitter(Vector(0, 0, 0), Vector(0, 0, 0), 0.25, 1.0))
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->add(1);
			force = CreateSimpleGravityForce(pba::Vector(0.0, -1.0, 0.0));
			GISolver solvera = CreateAdvancePosition(state, collisions);
			GISolver solverb = CreateAdvanceVelocity(state, force);
			solver = CreateLeapFrogSolver(solvera, solverb);
			std::cout << name << "constructed\n";
		}
		~BouncingBallsThing() {}

		void Init(const std::vector<std::string>& args) {}

		// Callback function
		void Display()
		{
			pba::Display(box);

			glPointSize(5.0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);

				glColor4f(ci.red(), ci.green(), ci.blue(), 1.0f);
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();
		}

		void Keyboard( unsigned char key, int x, int y )
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'v') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'e') { emit = !emit; }
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
			if (key == 'c')
			{
				box->set_coeff_restitution(box->coeff_restitution() / 1.1);
				std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
			}
			if (key == 'C')
			{
				box->set_coeff_restitution(box->coeff_restitution() * 1.1);
				std::cout << "coefficient of restituion: " << box->coeff_restitution() << std::endl;
			}
			if (key == 's')
			{
				box->set_coeff_sticky(box->coeff_sticky() / 1.1);
				std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
			}
			if (key == 'S')
			{
				box->set_coeff_sticky(box->coeff_sticky() * 1.1);
				std::cout << "coefficient of sticky: " << box->coeff_sticky() << std::endl;
			}
			if (key == 'l')
			{
				GISolver solvera = CreateAdvancePosition(state, collisions);
				GISolver solverb = CreateAdvanceVelocity(state, force);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Using Leap Frog solver" << std::endl;
			}
			if (key == 'f')
			{
				GISolver solvera = CreateAdvancePosition(state, collisions);
				GISolver solverb = CreateAdvanceVelocity(state, force);
				solver = CreateForwardEulerSolver(solvera, solverb);
				std::cout << "Using Forward Euler solver" << std::endl;
			}
			if (key == 'm')
			{
				pba::combineCollisionSurface(box, boxS);
				std::cout << "Create another smaller cube inside the scene" << std::endl;
			}
		}

		void solve()
		{
			if (emit)
			{
				int nbincrease = 10;
				state->add(nbincrease);
				Vector P, V;
				Color C;
				std::cout << "Emit Points" << state->nb() << std::endl;
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					emitter.emit(P, V, C);
					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);
				}
			}
			solver->solve(dt);
		}

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
		}

		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "v            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "e            toggle particle emission on/off\n";
//			cout << "k            toggle collision trace tree on/off\n";
			cout << "c/C          reduce/increase coefficient of restitution\n";
			cout << "l            use leap frog solver\n";
			cout << "f            use forward euler solver\n";
// 			cout << "b            use backward euler solver\n";
// 			cout << "6            use sixth order sover\n";
			cout << "m            create another smaller cube inside the scene\n";
		};

		void AddCollisionSurface(pba::CollisionSurface& s, pba::CollisionSurface& s2)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			boxS = s2;
			collisions.set_collision_surface(box);
		}

	private:
		bool emit;

		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;

		pba::CollisionSurface box;
		pba::CollisionSurface boxS;
		pba::ElasticCollisionHandler collisions;
		pba::ParticleEmitter emitter;
	};

	pba::PbaThing BouncingBalls() { return PbaThing(new BouncingBallsThing()); }
}