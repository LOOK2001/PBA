#include <iostream>

#include "PbaViewer.h"
#include "PbaUtils.h"
#include "things/RotatingCube.h"
#include "things/BasicBoid.h"
#include "things/BouncingBalls.h"
#include "things/SphInATeapot.h"

#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include <vector>
#include <String>

int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(1.0);
	{
		cube->get_triangle(4)->set_invisable();
		cube->get_triangle(5)->set_invisable();
	}
	CollisionSurface cubeS = pba::GenerateCollisionCube(0.15);

	BouncingBallsThing* BouncingBalls = new BouncingBallsThing;
	BouncingBalls->AddCollisionSurface(cube, cubeS);
	PbaThing balls = PbaThing(BouncingBalls);

	viewer->AddThing(balls);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++ ){
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}