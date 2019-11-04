#include <iostream>

#include "PbaViewer.h"
#include "PbaUtils.h"
#include "things/RotatingCube.h"
#include "things/BasicBoid.h"
#include "things/BouncingBalls.h"
#include "things/SphInATeapot.h"
#include "things/BouncingRBD.h"
#include "common/ObjParser.h"

#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include <vector>
#include <String>

int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(2.0);
	{
		cube->get_triangle(4)->set_invisable();
		cube->get_triangle(5)->set_invisable();
	}

	BouncingRBDThing* BouncingBalls = new BouncingRBDThing;
	//BouncingBallsThing* BouncingBalls = new BouncingBallsThing;
	BouncingBalls->AddCollisionSurface(cube);
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

//{
//	ObjParser* objReader = new ObjParser();
//	objReader->ParseFile("C:/Xicheng/MyLife/College/Code/PBA/pba2/pba2/common/utah_teapot2.obj");
//	CollisionSurface teapot = makeCollisionSurface();
//	objReader->Fill(teapot);
//	teapot->toggle_wireframe();
//
//	BouncingBallsThing* BouncingBalls = new BouncingBallsThing;
//	SphInATeapotThing* BouncingBalls = new SphInATeapotThing;
//	BouncingBalls->AddCollisionSurface(cube);
//	PbaThing balls = PbaThing(BouncingBalls);
//}