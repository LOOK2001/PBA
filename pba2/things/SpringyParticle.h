#pragma once

#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
//#include "ExplicitDynamics.h"
//#include "RK4.h"
#include "GISolver.h"
#include "ForceLibrary.h"
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
	class SpringyThing : public PbaThingyDingy
	{
	public:
		SpringyThing(const std::string nam = "SpringyThing") : PbaThingyDingy(nam), display_pp(false)
		{
			state = CreateDynamicalState(name + "DynamicalData");
			state->create_attr("home", pba::Vector(0, 0, 0));
			state->add(1);
			//force = pba::CreateSpringForce(1.0, pba::Vector(0, 0, 0), 1.0);
		}
		~SpringyThing() {}
		void Init(const std::vector<std::string>& args) {}

		void Display()
		{
			glPointSize(15.0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < state->nb(); i++)
			{
				const Color& ci = state->ci(i);
				const Vector& v = state->pos(i);
				glColor3f(ci.red(), ci.green(), ci.blue());
				glVertex3f(v.X(), v.Y(), v.Z());
			}
			glEnd();
		}
		void solve() { solver->solve(dt); }
	private:
		bool display_pp;

		pba::DynamicalState state;
		pba::Force force;
		pba::GISolver solver;
	};

	pba::PbaThing Springy() { return PbaThing(new SpringyThing()); }
}