#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"
#include "DynamicalState.h"
//#include "SPHSolver.h"
//#include "RK4.h"
#include "ForceLibrary.h"
//#include "SPHForce.h"
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
			emitter(ParticleEmitter(Vector(50.5, -10.0, 1.5), Vector(-1.0, 0, 0), 2.5, 10.0))
		{

		}
		~SphInATeapotThing() {}
	private:
		bool emit;

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