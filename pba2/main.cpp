#include <iostream>

#include "PbaViewer.h"
#include "PbaUtils.h"
#include "things/RotatingCube.h"
#include "things/BasicBoid.h"
#include "things/BouncingBalls.h"
#include "things/SphInATeapot.h"
#include "things/BouncingRBD.h"
#include "things/SBDAreaCloth.h"
#include "things/BouncingSpheres.h"

#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include <vector>
#include <String>

#define SPHERE_COLLISION
//#define AS_05
//#define AS_04
//#define AS_03

#ifdef SPHERE_COLLISION
int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	PbaThing Sph = BouncingSpheres();

	viewer->AddThing(Sph);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++) {
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}
#endif // SPHERE_COLLISION

#ifdef AS_05
int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	PbaThing SBD = SBDAreaCloth("Cloth");

	viewer->AddThing(SBD);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++) {
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}
#endif // AS_05

#ifdef AS_04
int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(2.0);
	{
		cube->get_triangle(4)->set_invisible();
		cube->get_triangle(5)->set_invisible();
	}

	ObjParser* objReader = new ObjParser();
	objReader->ParseFile("C:/Xicheng/MyLife/College/Code/PBA/pba2/pba2/common/bunny.obj");

	BouncingRBDThing* BouncingBalls = new BouncingRBDThing;
	BouncingBalls->AddCollisionSurface(cube);

	objReader->Fill(BouncingBalls->get_State());
	BouncingBalls->Reset();
	PbaThing balls = PbaThing(BouncingBalls);

	viewer->AddThing(balls);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++) {
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}
#endif // AS_04

#ifdef AS_03
int main(int argc, char* argv[])
{
	using namespace pba;
	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(2.0);
	{
		cube->get_triangle(4)->set_invisible();
		cube->get_triangle(5)->set_invisible();
	}

	ObjParser* objReader = new ObjParser();
	objReader->ParseFile("C:/Xicheng/MyLife/College/Code/PBA/pba2/pba2/common/utah_teapot2.obj");
	CollisionSurface teapot = makeCollisionSurface();
	objReader->Fill(teapot);
	teapot->toggle_wireframe();

	BouncingBallsThing* BouncingBalls = new BouncingBallsThing;
	//SphInATeapotThing* BouncingBalls = new SphInATeapotThing;
	BouncingBalls->AddCollisionSurface(cube);
	PbaThing balls = PbaThing(BouncingBalls);

	viewer->AddThing(balls);
	std::vector<string> vec;
	for (int i = 0; i < argc; i++) {
		vec.push_back(argv[i]);
	}
	viewer->Init(vec);

	viewer->MainLoop();

	std::cout << "Hello" << std::endl;

	return 1;
}
#endif // AS_03