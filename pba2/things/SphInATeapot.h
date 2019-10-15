#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
#include "SPHSolver.h"
//#include "RK4.h"
#include "ForceLibrary.h"
#include "SPHForce.h"
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
	class SphInATeapotThing : public PbaThingyDingy
	{
	public:
		SphInATeapotThing(const std::string nam = "SphInATeapotThing") :
			PbaThingyDingy(nam),
			emit(false),
			box(pba::makeCollisionSurface()),
			emitter(ParticleEmitter(Vector(0.0, 0.0, 0.0), Vector(-1.0, 0, 0), 0.05, 10.0))
		{
			AABB bounds(Vector(-3, -3, -3), Vector(3, 3, 3));
			double h = 0.5/*3.0 / 200.0*/;
			state = CreateSPH(bounds, h, name + "DynamicalData");
			//state->add(280000/10);

			force = CreateAccumulatingForce();
			gravityforce = CreateAccumulatingGravityForce(pba::Vector(0, -10.0, 0));
			sphforce = CreateSPHForce();

			std::shared_ptr<AccumulatingForce> f = dynamic_pointer_cast<AccumulatingForce>(force);
			f->add(gravityforce);
			f->add(sphforce);
			solvera = CreateAdvancePosition(state, collisions);
			solverb = CreateAdvanceVelocity(state, force, 5.6, 120.0);
			//solver = CreateForwardEulerSolver(solvera,solverb);
			solver = CreateLeapFrogSolver(solvera, solverb);
			//solver = CreateGISolverSixthOrder(solver);
			Reset();
			std::cout << name << " constructed\n";

			double test1 = (-1.0 / 0.0) * 0.0;
			double test2 = test1 * 0.0;
			if (isnan(test1))
			{
				int test3 = 123;
			}
		}
		~SphInATeapotThing() {}

		// Callback functions
		void Display()
		{
			pba::Display(box);

			glPointSize(3.5);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const pba::Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();
		};

		void Keyboard(unsigned char key, int x, int y)
		{
			PbaThingyDingy::Keyboard(key, x, y);
			if (key == 'v') { box->toggle_visible(); }
			if (key == 'w') { box->toggle_wireframe(); }
			if (key == 'e') { emit = !emit; }
		}

		void solve()
		{
			if (emit)
			{
				int nbincrease = 50;
				state->add(nbincrease);
				std::cout << "Emit: Total Points " << state->nb() << std::endl;
//#pragma omp parallel for
				for (size_t i = state->nb() - nbincrease; i < state->nb(); i++)
				{
					Vector P, V;
					Color C;
//#pragma omp critical
					{
						emitter.emit(P, V, C);
					}
					C[0] = 0.8 + 0.2 * C[0];

					//P = Vector(0.0, 0.0, 0.0);
					//V = Vector(0.0, 0.0, 0.0);

					state->set_pos(i, P);
					state->set_vel(i, V);
					state->set_ci(i, C);
				}
				emit = !emit;
			}
			solver->solve(dt);
		};

		void Reset()
		{
			//state->clear();
		}

		void AddCollisionSurface(pba::CollisionSurface& s)
		{
			std::cout << "Add CollisionSurface\n";
			//std::cout << "AABB: [" << s->aabb().LLC().X() << ", " << s->aabb().LLC().Y() << ", " << s->aabb().LLC().Z() << "] X [" << s->aabb().URC().X() << ", " << s->aabb().URC().Y() << ", " << s->aabb().URC().Z() << "]" << std::endl;
			box = s;
			collisions.set_collision_surface(box);
		}

	private:
		bool emit;

		pba::SPHState state;
		pba::Force force;
		pba::Force sphforce;
		pba::Force gravityforce;
		pba::GISolver solver;
		pba::GISolver solvera;
		pba::GISolver solverb;

		pba::CollisionSurface box;
		pba::ElasticCollisionHandler collisions;
		pba::ParticleEmitter emitter;

		float vclamp;
	};

	pba::PbaThing SphInATeapot() { return PbaThing(new SphInATeapotThing()); }
}