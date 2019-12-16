#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "SoftBodyState.h"
#include "SBDSolver.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "ForceLibrary.h"
//#include "PoincareData.h"
#include "CollisionSurface.h"
#include "CollisionHandler.h"
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
	class ClothSphereGameThing : public PbaThingyDingy
	{
	public:
		ClothSphereGameThing(const std::string nam = "BouncingSpheresThing") :
			PbaThingyDingy(nam),
			display_pp(false),
			emit(false),
			fire(false),
			initial_velocity(pba::Vector(0.0)),
			initial_position(pba::Vector(0.0))
		{
			{
				SBDstate = CreateSoftBody(name + "SoftBodyData");
				SBDstate->create_attr("pscale", 0.17f);
				SBDforce = CreateAccumulatingForce();
				gforce = CreateAccumulatingGravityForce(pba::Vector(0, -5.0, 0));
				strutforce = CreateAccumulatingStrutForce(1400.0, 1.0);
				strutareaforce = CreateAccumulatingStrutAreaForce(1500.0, 26.0);
				strutbendforce = CreateAccumulatingStrutBendForce(15.0, 30.0);
				std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(SBDforce);
				f->add(strutforce);
				f->add(strutareaforce);
				f->add(strutbendforce);
				f->add(gforce);
				std::cout << "Forces constructed\n";

				SBDcollisions.set_CR(0.995);
				GISolver solvera = CreateAdvanceRotation(SBDstate, SBDcollisions);
				GISolver solverb = CreateAdvanceAngularVelocity(SBDstate, SBDforce);

				nx = 7;
				ny = 2;
				{
					solvera->addFixedId(0);
					solvera->addFixedId(ny);
					solvera->addFixedId((nx) * (ny + 1));
					solvera->addFixedId((nx + 1) * (ny + 1) - 1);

					solverb->addFixedId(0);
					solverb->addFixedId(ny);
					solverb->addFixedId((nx) * (ny + 1));
					solverb->addFixedId((nx + 1) * (ny + 1) - 1);
				}

				pba::GISolver basicsolver = CreateLeapFrogSolver(solvera, solverb);
				SBDsolver = CreateGISolverSubstep(basicsolver, 9);

				score = 0;
				hitNum = 0;
			}

			std::cout << name << " constructed\n";
		}
		~ClothSphereGameThing() {};

		void Init(const std::vector<std::string>& args)
		{
			// generate the plane
			CollisionSurface surf = pba::GenerateCollisionPlane(Vector(3.0, 1.0, 8.0));
			for (size_t i = 0; i < surf->triangle_size(); i++)
				surf->get_triangle(i)->set_color(Color(0.3f, 0.3f, 0.9f, 1.0f));

			// generate the target
			CollisionSurface box = pba::GenerateCollisionCube(0.45, Vector(0.0, 0.0, 7.5));
			for (size_t i = 0; i < box->triangle_size(); i++)
			{
				box->get_triangle(i)->set_color(Color(0.9f, 0.3f, 0.3f, 1.0f));
				box->get_triangle(i)->set_collision_type(CollisionTriangleRaw::TARGET);
				if (i == 4 || i == 5)
					continue;
				box->get_triangle(i)->set_invisible();
			}
			target = box;
			combineCollisionSurface(surf, target);
			AddCollisionSurface(surf);

			AddStateGeometry(pba::Vector(-2.5, 0.0, 0), pba::Vector(2.5, 0.8, 0));

			toggleAnimate();
		}

		// Callback functions
		void Display()
		{
			//setlight();
			//setmaterial();

			pba::Display(box);
			pba::Display(target);

			glBegin(GL_LINES);
			glLineWidth(10.0f);
			const Color ci(1.0, 1.0, 1.0, 1.0);
			glColor3f(ci.red(), ci.green(), ci.blue());
			for (size_t i = 0; i < SBDstate->nb_pairs(); i++)
			{
				const pba::SoftEdge& e = SBDstate->get_connected_pair(i);
				const pba::Vector& v1 = SBDstate->pos(e->get_first_node());
				glVertex3f(v1.X(), v1.Y(), v1.Z());
				const pba::Vector& v2 = SBDstate->pos(e->get_second_node());
				glVertex3f(v2.X(), v2.Y(), v2.Z());
			}
			glEnd();

			for (size_t i = 0; i < SBDstate->nb(); i++)
			{
				const Color& ci = SBDstate->ci(i);
				const pba::Vector& v = SBDstate->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glTranslatef(v.X(), v.Y(), v.Z());
				glutSolidSphere(SBDstate->get_float_attr("pscale", i), 20, 10);
				glTranslatef(-v.X(), -v.Y(), -v.Z());
			}
			glEnd();
		}

		void AddStateGeometry(const pba::Vector& LLC, const pba::Vector& URC)
		{
			std::cout << "ASG start\n";
			home_state = GeneratePlanarSoftBody(LLC, URC, nx, ny, false);
			for (size_t i = 0; i < home_state->nb(); i++)
			{
				Color C(drand48(), drand48(), drand48(), 1.0);
				home_state->set_ci(i, C);
				home_state->set_vel(i, initial_velocity);
			}
			std::cout << "ASG reset\n";
			Reset();
			std::cout << "ASG end\n";
		}

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'j')
			{
				fire = !fire;
				velocity_factor = 1.0f;
			}
			if (key == 'm')
			{
				mass_factor = mass_factor / 1.1;
				std::cout << "mass_factor: " << mass_factor << std::endl;
			}
			if (key == 'M')
			{
				mass_factor = mass_factor * 1.1;
				std::cout << "mass_factor: " << mass_factor << std::endl;
			}
		};

		void Mouse(int b, int state, int x, int y)
		{
			PbaThingyDingy::Mouse(b, state, x, y);
			if (b == GLUT_LEFT_BUTTON)
			{
				if (state == GLUT_UP)
				{
					fire = !fire;
					velocity_factor = PbaViewer::Instance()->getPressTime();
				}
			}
		}

		void solve()
		{
			if (fire)
			{
				// fire
				int nbincrease = 1;
				hitNum++;
				SBDstate->add(nbincrease);
				Vector P, V;
				Color C = Color(velocity_factor * 0.6, 1 - velocity_factor * 0.5, 1 - velocity_factor * 0.5, 1.0);
				for (size_t i = SBDstate->nb() - nbincrease; i < SBDstate->nb(); i++)
				{
					P = PbaViewer::Instance()->getCamerPos();
					V = PbaViewer::Instance()->getCamerDir();
					SBDstate->set_pos(i, P);
					SBDstate->set_vel(i, V * 10.0f * velocity_factor);
					SBDstate->set_ci(i, C);
					SBDstate->set_attr("pscale", i, 0.4f);
					SBDstate->set_mass(i, mass_factor);
				}
				fire = !fire;
			}

			SBDsolver->solve(dt);
		};

		void Reset()
		{
			SBDstate->clear();
			SBDstate->clear_pairs();
			SBDstate->add(home_state->nb());

			for (size_t i = 0; i < home_state->nb(); i++)
			{
				SBDstate->set_ci(i, home_state->ci(i));
				SBDstate->set_pos(i, home_state->pos(i));
				SBDstate->set_vel(i, home_state->vel(i));
			}
			for (size_t i = 0; i < home_state->nb_pairs(); i++)
			{
				const SoftEdge& e = home_state->get_connected_pair(i);
				SBDstate->add_pair(e->get_first_node(), e->get_second_node());
			}
			for (size_t i = 0; i < home_state->nb_area_sets(); i++)
			{
				const SoftTriangle& e = home_state->get_area_set(i);
				SBDstate->add_triangle(e->get_first_node(), e->get_second_node(), e->get_third_node());
			}
			for (size_t i = 0; i < home_state->nb_bendables(); i++)
			{
				const SoftBendable& e = home_state->get_bendable(i);
				SBDstate->add_bend(e->get_first_node(), e->get_second_node(), e->get_third_node(), e->get_fourth_node());
			}
			std::cout << "Nb points " << SBDstate->nb() << "    Nb pairs " << SBDstate->nb_pairs() << "    Nb Area Sets " << SBDstate->nb_area_sets() << "   Nb bend pairs " << SBDstate->nb_bendables() << std::endl;
		};

		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "SPACEBAR     start/stop animation\n";
			cout << "t/T          reduce/increase animation time step\n";
			cout << "g/G          reduce/increase gravitational constant\n";
		};

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			box = s;
			box->set_coeff_restitution(0.65);
			box->set_coeff_sticky(1.0);
			SBDcollisions.set_collision_surface(box);
		}

		void messageEvent(unsigned int _message)
		{
			PbaThingyDingy::messageEvent(_message);

			switch (_message)
			{
			case PBA_HIT_TARGET:
				double _x = (drand48() - 0.5) * 4;
				double _y = drand48() * 0.5;
				target->translate(Vector(_x, _y, 0.0));
				score++;
				if (score >= 3)
				{
					endGame();
				}
				break;
			}
		}

		void endGame()
		{
			float hitRate = (float)score / (float)hitNum;
			std::cout << "----------------------------------" << std::endl;
			std::cout << "score: " << score << std::endl;
			std::cout << "Shots: " << hitNum << std::endl;
			std::cout << "Hit rate: " << hitRate << std::endl;

			toggleAnimate();
		}

	private:
		bool display_pp;
		bool emit;
		bool fire;

		pba::CollisionSurface box;

		pba::SoftBodyState SBDstate;
		pba::SoftBodyState home_state;
		pba::Force SBDforce;
		pba::Force gforce;
		pba::Force strutforce;
		pba::Force strutareaforce;
		pba::Force strutbendforce;
		pba::GISolver basicsolver;
		pba::GISolver SBDsolver;
		pba::ElasticSBDSphereCollisionHandler SBDcollisions;
		pba::CollisionSurface target;

		int hitNum = 0;
		int score;
		double velocity_factor = 0.0f;
		float mass_factor = 1.0;

		pba::Vector initial_velocity;
		pba::Vector initial_position;

		int nx, ny;
	};

	pba::PbaThing ClothSphereGame() { return PbaThing(new ClothSphereGameThing()); }
}