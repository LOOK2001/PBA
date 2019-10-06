#include <iostream>

#include <omp.h>
#include "PbaViewer.h"
#include "PbaUtils.h"
#include "things/RotatingCube.h"
#include "things/BasicBoid.h"
#include "things/BouncingBalls.h"
#include "things/ParticleOnASphere.h"

#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include <vector>
#include <String>

#define _DEBUG
//#define _CONST_AND_BOID

int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	// Flocking and Constraint System
	ConstraintAndBoidForceThing* parOnSph = new ConstraintAndBoidForceThing(0.1, 1.0, 1.0, 10, 5.0, 3.0, 5.0);
	PbaThing assignment = PbaThing(parOnSph);

	viewer->AddThing(assignment);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++ ){
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}