#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "RigidBodyState.h"
#include "RBDSolver.h"
//#include "RK4.h"
#include "ForceLibrary.h"
//#include "PoincareData.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
#include "PbaUtils.h"
#include "LinearAlgebra.h"

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

	class BouncingRBDThing : public PbaThingyDingy
	{
	public:

		BouncingRBDThing(const std::string nam = "BouncingRBDThing") :
			PbaThingyDingy(nam),
			display_pp(false),
			collision_visible(true),
			box(pba::makeCollisionSurface()),
			x_aspect(1.0),
			y_aspect(1.0)
		{
			state = CreateRigidBody(name + "RigidBodyData");
			state->add(10);
			force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
			GISolver solvera = CreateAdvanceRotation(state, collisions);
			GISolver solverb = CreateAdvanceAngularVelocity(state, force);
			//basicsolver = CreateLeapFrogSolver(solvera, solverb);
			//basicsolver = CreateGISolverSixthOrder(basicsolver);
			//solver = CreateGISolverSubstep(basicsolver, 1);
			solver = CreateLeapFrogSolver(solvera, solverb);
			Reset();
			std::cout << name << " constructed\n";
		};
		~BouncingRBDThing() {};

		void Init(const std::vector<std::string>& args)
		{
			std::cout << "Init " << name << std::endl;
		}

		// Callback functions
		void Display()
		{
			if (display_pp)
			{
// 				glPointSize(5.0);
// 				glBegin(GL_LINES);
// 				glColor3f(1.0, 1.0, 1.0);
// 				for (size_t i = 0; i < poincare.size(); i++)
// 				{
// 					const float q = poincare.ValueX(i);
// 					const float p = poincare.ValueY(i);
// 					glVertex2f(q, p);
// 				}
// 				glEnd();
			}
			else
			{
				pba::Display(box);

				glPointSize(5.0);
				glBegin(GL_POINTS);
				for (size_t i = 0; i < state->nb(); i++)
				{
					const Color& ci = state->ci(i);
					const pba::Vector& v = state->vert_pos(i);
					glColor3f(ci.red(), ci.green(), ci.blue());
					glVertex3f(v.X(), v.Y(), v.Z());
				}
				glEnd();
			}
		};

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'p') { display_pp = !display_pp; }
			if (key == 'v') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'k') { collisions.toggle_tree(); }
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

			if (key == '1' || key == '2' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9')
			{
				//solver = CreateGISolverSubstep(basicsolver, int(key) - 48);
				//std::cout << "Switched to solver with " << int(key) - 48 << " substeps\n";
			}

			if (key == 'm')
			{
				state->set_mass(0, state->mass(0) / 1.1);
				state->compute_M();
				std::cout << "Mass of particle 0: " << state->mass(0) << std::endl;
			}
			if (key == 'M')
			{
				state->set_mass(0, state->mass(0) * 1.1);
				state->compute_M();
				std::cout << "Mass of particle 0: " << state->mass(0) << std::endl;
			}

			if (key == 'x')
			{
				x_aspect /= 1.1;
				std::cout << "X Aspect: " << x_aspect << std::endl;
			}
			if (key == 'X')
			{
				x_aspect *= 1.1;
				std::cout << "X Aspect: " << x_aspect << std::endl;
			}
			if (key == 'y')
			{
				y_aspect /= 1.1;
				std::cout << "Y Aspect: " << y_aspect << std::endl;
			}
			if (key == 'Y')
			{
				y_aspect *= 1.1;
				std::cout << "Y Aspect: " << y_aspect << std::endl;
			}

		};
		void solve()
		{
			solver->solve(dt);
		};
		void Reset()
		{
			// Distribute particles with random positions
			Vector zero(0, 0, 0);
			for (size_t i = 0; i < state->nb(); i++)
			{
				Vector P(x_aspect * (drand48() - 0.5), y_aspect * (drand48() - 0.5), drand48() - 0.5);
				P *= 2.5;
				Color C(drand48(), drand48(), drand48(), 1.0);
				state->set_pos(i, P);
				state->set_ci(i, C);
				state->set_vel(i, zero);
			}
			state->compute_RBD_data();
			state->center_of_mass = Vector(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
			state->linear_velocity = Vector(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
			state->angular_velocity = Vector(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
			state->angular_momentum = state->inertia_moment() * state->angular_velocity;
		};
		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "v            toggle visibility of collision surface\n";
			cout << "w            toggle wireframe/solid display of collision surface\n";
			cout << "g/G          reduce/increase gravitational constant\n";
			cout << "1-9          select number of sover substeps\n";
			cout << "k            toggle collision trace tree on/off\n";
			cout << "m/M          reduce/increase mass of particle 0\n";
			cout << "x/X          reduce/increase x aspect ratio of rigid body\n";
			cout << "y/Y          reduce/increase y aspect ratio of rigid body\n";
		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			collisions.set_collision_surface(box);
		}

	private:

		bool display_pp;
		bool collision_visible;

		pba::RigidBodyState state;
		pba::Force force;
		pba::GISolver basicsolver;
		pba::GISolver solver;

		pba::CollisionSurface box;
		pba::ElasticRBDCollisionHandler collisions;

		float x_aspect, y_aspect;

	};

	pba::PbaThing BouncingRBD() { return PbaThing(new BouncingRBDThing()); }
}