#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "ConstraintLibrary.h"
#include "DynamicalState.h"
#include "ExplicitDynamics.h"
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
	class ConstraintAndBoidForceThing : public PbaThingyDingy
	{
	public:
		ConstraintAndBoidForceThing(const double avoid, const double match, const double center, const int nb, const double amax, const double rng, const double rng_ramp, const std::string nam = "ParticleOnASphereThing") :
			PbaThingyDingy(nam),
			constraint(pba::CreateMultiConstraint()),
			emitter(ParticleEmitter(Vector(0, 1, 0), Vector(0, 0, 0), 0.01, 0.1)),
			emit(false),
			constraint_is_visible(true),
			report(false),
			showSolidSphere(false)
		{
			leadBoidID = 0;
			state = CreateDynamicalState(name + "DynamicalData");
			state->add(nb);

			for (size_t i = 0; i < state->nb(); i++)
			{
				Constraint pcon = CreateParticleOnSphereConstraint(1.0, Vector(0, 0, 0), i);
				pcon->set_Ks(1.0);
				pcon->set_Kf(1.0);
				constraint->addConstraint(pcon);
			}

#define _CONST_AND_BOID
#ifdef _CONST_AND_BOID
			boid_force = CreateAccumulatingBoidForce(avoid, match, center, amax, rng, rng_ramp);
			// set lead boid
			leadBoidID = 0;
			std::shared_ptr<AccumulatingBoidForce> boidF = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
			boidF->set_lead_boid_id(leadBoidID);

			force = CreateAccumulatingForce();
			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);
			f->add(boid_force);
#else
			force = CreateSimpleGravityForce(pba::Vector(0, -1.0, 0));
#endif // _CONST_AND_BOID

			//GISolver solvera = CreateAdvancePosition(state, constraint);
			// AdvancePosition Solver
			GISolver solvera = CreateAdvancePosition(state);
			// AdvanceVelocityWithConstraint Solver
			GISolver solverb = CreateAdvanceVelocity(state, force, constraint);

			solver = CreateLeapFrogSolver(solvera, solverb);
			Reset();
			std::cout << name << " constructed\n";
		};
		~ConstraintAndBoidForceThing() {};

		void Init(const std::vector<std::string>& args)
		{}

		// Callback functions
		void Display()
		{
			if (constraint_is_visible)
			{
				glColor3f(1.0, 1.0, 1.0);
				glutWireSphere(1.0, 20, 20);
				if (showSolidSphere)
				{
					glColor3f(0.2, 0.2, 0.2);
					glutSolidSphere(0.98, 20, 20);
				}
			}

#ifndef _DEBUG
			for (int ii = 0; ii < lines.size(); ii++)
			{
				glLineWidth(3);
				glLineStipple(1, 0xFF0C);
				glBegin(GL_LINES);
				pointA = lines[ii].first;
				pointB = lines[ii].second;
				glVertex3f(pointA[0], pointA[1], pointA[2]);
				glVertex3f(pointB[0], pointB[1], pointB[2]);
				glEnd();
				glLineWidth(1);
			}
			lines.clear();
#endif // _DEBUG

			glPointSize(10.0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++){
				if (i == leadBoidID) {
					glEnd();
					glPointSize(20.0);
					glBegin(GL_POINTS);

					const Color& ci = state->ci(i);
					const pba::Vector& v = state->pos(i);
					glColor3f(ci.red(), ci.green(), ci.blue());
					glVertex3f(v.X(), v.Y(), v.Z());

					glEnd();
					glPointSize(5.0);
					glBegin(GL_POINTS);
					continue;
				}
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();

			if (report)
			{
				for (size_t i = 0; i < state->nb(); i++)
				{
					std::cout << "Magnitude for particle " << i << " " << state->pos(i).magnitude() << std::endl;
				}
			}
		}

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'R') { report = !report; }
			if (key == 'e') { emit = !emit; }
			if (key == 'a')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_avoidance(f->get_avoidance() / 1.1);
				std::cout << "Collision Avoidance Strength: " << f->get_avoidance() << std::endl;
			}
			if (key == 'A')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_avoidance(f->get_avoidance() * 1.1);
				std::cout << "Collision Avoidance Strength: " << f->get_avoidance() << std::endl;
			}
			if (key == 'v')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_matching(f->get_matching() / 1.1);
				std::cout << "Velocity Matching Strength: " << f->get_matching() << std::endl;
			}
			if (key == 'V')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_matching(f->get_matching() * 1.1);
				std::cout << "Velocity Matching Strength: " << f->get_matching() << std::endl;
			}
			if (key == 'c')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_centering(f->get_centering() / 1.1);
				std::cout << "Centering Strength: " << f->get_centering() << std::endl;
			}
			if (key == 'C')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_centering(f->get_centering() * 1.1);
				std::cout << "Centering Strength: " << f->get_centering() << std::endl;
			}
			if (key == 'm')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_max(f->get_max() / 1.1);
				std::cout << "Maximum Acceleration: " << f->get_max() << std::endl;
			}
			if (key == 'M')
			{
				std::shared_ptr<AccumulatingBoidForce> f = dynamic_pointer_cast<AccumulatingBoidForce>(boid_force);
				f->set_max(f->get_max() * 1.1);
				std::cout << "Maximum Acceleration: " << f->get_max() << std::endl;
			}
			if (key == 's')
			{
				double Ks = constraint->get_constraint(0)->get_Ks();
				Ks -= 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Ks(Ks);
				}
				std::cout << "Constraint Ks " << Ks << std::endl;
			}
			if (key == 'S')
			{
				double Ks = constraint->get_constraint(0)->get_Ks();
				Ks += 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Ks(Ks);
				}
				std::cout << "Constraint Ks " << Ks << std::endl;
			}
			if (key == 'd')
			{
				double Kf = constraint->get_constraint(0)->get_Kf();
				Kf -= 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Kf(Kf);
				}
				std::cout << "Constraint Kf " << Kf << std::endl;
			}
			if (key == 'D')
			{
				double Kf = constraint->get_constraint(0)->get_Kf();
				Kf += 0.001;
				for (size_t c = 0; c < constraint->nb(); c++)
				{
					constraint->get_constraint(c)->set_Kf(Kf);
				}
				std::cout << "Constraint Kf " << Kf << std::endl;
			}
			if (key == '1')
			{
				// Relaxation only
				GISolver solvera = CreateAdvancePosition(state);
				GISolver solverb = CreateAdvanceVelocity(state, force, constraint);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Relaxation only " << std::endl;
			}
			if (key == '2')
			{
				// Position-Baed Only
				GISolver solvera = CreateAdvancePosition(state, constraint);
				GISolver solverb = CreateAdvanceVelocity(state, force);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Position-Based Only " << std::endl;
			}
			if (key == '3')
			{
				// Mixed Position-Based and Relaxation
				GISolver solvera = CreateAdvancePosition(state, constraint);
				GISolver solverb = CreateAdvanceVelocity(state, force, constraint);
				solver = CreateLeapFrogSolver(solvera, solverb);
				std::cout << "Mixed Position-Based and Relaxation " << std::endl;
			}
			if (key == '4')
			{
				showSolidSphere = !showSolidSphere;
			}
			if (key == 'H')
			{
				Usage();
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
				std::cout << "Emit Points " << state->nb() << std::endl;
				double Ks = constraint->get_constraint(0)->get_Ks();
				double Kf = constraint->get_constraint(0)->get_Kf();
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					C = Color(drand48(), drand48(), drand48(), 1);
					// Random position
					double s = 2.0 * drand48() - 1.0;
					double phi = drand48() * 2.0 * 3.14159265;
					double ss = std::sqrt(1.0 - s * s);
					P = Vector(ss * std::cos(phi), s - 0.1, ss * std::sin(phi));
					P = P.unitvector();
					V = Vector(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
					V -= ((V * P) / (P * P)) * P;
					V *= (1.0 + drand48() * 3.0) / 2.0;

					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);

					Constraint pcon = CreateParticleOnSphereConstraint(1.0, Vector(0, 0, 0), (int)i);
					pcon->set_Ks(Ks);
					pcon->set_Kf(Kf);
					constraint->addConstraint(pcon);

					emit = false;
				}
			}
			solver->solve(dt);
		};

		void Reset()
		{
			// Distribute particles with random positions
			for (size_t i = 0; i < state->nb(); i++)
			{
				Color C(drand48(), drand48(), drand48(), 1);
				double s = 2.0 * drand48() - 1.0;
				double phi = drand48() * 2.0 * 3.14159265;
				double ss = std::sqrt(1.0 - s * s);
				Vector P(ss * std::cos(phi), s - 0.1, ss * std::sin(phi));
				P = P.unitvector();
				Vector V(drand48() - 0.5, drand48() - 0.5, drand48() - 0.5);
				V -= ((V * P) / (P * P)) * P;
				V *= (1.0 + drand48() * 3.0) / 2.0;
				state->set_pos(i, P);
				state->set_vel(i, V);
				state->set_ci(i, C);
			}
		};

		void Usage()
		{
			PbaThingyDingy::Usage();
			cout << "=== " << name << " ===\n";
			cout << "e            toggle emitting particles\n";
			cout << "-------------Boid Force-------------\n";
			cout << "a/A          reduce/increase collision avoidance force constant\n";
			cout << "v/V          reduce/increase velocity matching force constant\n";
			cout << "c/C          reduce/increase centering force constant\n";
			cout << "m/M          reduce/increase maximum acceleration\n";
			cout << "-------------Constraint-------------\n";
			cout << "s/S          reduce/increase constraint spring force\n";
			cout << "d/D          reduce/increase constraint friction force\n";
			cout << "R            toggle reporting particle data on/off\n";
			cout << "1            Relaxation solver\n";
			cout << "2            Position-based solver\n";
			cout << "3            Mixed relaxation & position-based solver\n";
		};

	private:
		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;
		pba::MultiConstraint constraint;
		pba::ParticleEmitter emitter;
		bool emit;
		bool constraint_is_visible;

		pba::Force boid_force;
		int leadBoidID;

		bool report;
		bool showSolidSphere;
	};

	//pba::PbaThing ConstraintAndBoidForce() { return PbaThing(new ConstraintAndBoidForceThing()); }
}