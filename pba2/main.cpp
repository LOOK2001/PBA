#include <iostream>

#include "PbaViewer.h"
#include "PbaUtils.h"
#include "things/RotatingCube.h"
#include "things/BasicBoid.h"
#include "things/BouncingBalls.h"
#include "things/SphInATeapot.h"
#include "things/BouncingRBD.h"
#include "things/SBDAreaCloth.h"
#include "common/ObjParser.h"

#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include <vector>
#include <String>

#define AS_05
//#define AS_04
//#define AS_03

// 				const pba::SoftBendable& tri = state->get_bendable(i);
// 				const pba::Vector& v1 = state->pos(tri->get_first_node());
// 				glVertex3f(v1.X(), v1.Y(), v1.Z());
// 				const pba::Vector& v2 = state->pos(tri->get_second_node());
// 				glVertex3f(v2.X(), v2.Y(), v2.Z());
// 				const pba::Vector& v3 = state->pos(tri->get_third_node());
// 				glVertex3f(v3.X(), v3.Y(), v3.Z());
// 				const pba::Vector& v4 = state->pos(tri->get_fourth_node());
// 				glVertex3f(v4.X(), v4.Y(), v4.Z());

#ifdef AS_05
int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(1.0);

	SBDAreaClothThing* BouncingBalls = new SBDAreaClothThing("Cloth");
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
#endif // AS_05

#ifdef AS_04
int main(int argc, char* argv[]) {
	using namespace pba;

	PbaViewer* viewer = PbaViewer::Instance();

	CollisionSurface cube = pba::GenerateCollisionCube(2.0);
	{
		cube->get_triangle(4)->set_invisable();
		cube->get_triangle(5)->set_invisable();
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
		cube->get_triangle(4)->set_invisable();
		cube->get_triangle(5)->set_invisable();
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