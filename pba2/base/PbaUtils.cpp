#include "PbaUtils.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
#include <OpenGL/glu.h>  // GLU support library.
#include <GLUT/glut.h>
#else
#include <Windows.h>
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

void pba::AddCollisionSurface(CollisionSurface& s, PbaThing& p)
{
	return;
}

void pba::Display(CollisionSurface& s)
{
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < s->triangle_size(); i++)
	{
		if (!s->get_triangle(i)->visibility())
			continue;

		pba::Color color = s->get_triangle(i)->get_color();
		glColor4f(color.red(), color.green(), color.blue(), color.alpha());

		pba::Vector v = s->get_triangle(i)->vertex(0);
		glVertex3f(v.X(), v.Y(), v.Z());
		glNormal3f(v.X(), v.Y(), v.Z());

		v = s->get_triangle(i)->vertex(1);
		glVertex3f(v.X(), v.Y(), v.Z());
		glNormal3f(v.X(), v.Y(), v.Z());

		v = s->get_triangle(i)->vertex(2);
		glVertex3f(v.X(), v.Y(), v.Z());
		glNormal3f(v.X(), v.Y(), v.Z());
	}
	glEnd();
	return;
}

pba::CollisionSurface pba::GenerateCollisionCube(double size)
{
	// vertices
	pba::Vector verts[8];

	// faces
	std::vector< std::vector<int> > faces;

	// face colors
	pba::Color face_colors[6];

	verts[0] = pba::Vector(-1 * size, -1 * size, -1 * size);
	verts[1] = pba::Vector(1 * size, -1 * size, -1 * size);
	verts[2] = pba::Vector(1 * size, 1 * size, -1 * size);
	verts[3] = pba::Vector(-1 * size, 1 * size, -1 * size);
	verts[4] = pba::Vector(-1 * size, -1 * size, 1 * size);
	verts[5] = pba::Vector(1 * size, -1 * size, 1 * size);
	verts[6] = pba::Vector(1 * size, 1 * size, 1 * size);
	verts[7] = pba::Vector(-1 * size, 1 * size, 1 * size);

	face_colors[0] = pba::Color(1, 0, 1, 1);
	face_colors[1] = pba::Color(1, 0, 0, 1);
	face_colors[2] = pba::Color(0, 0, 1, 1);
	face_colors[3] = pba::Color(0, 1, 0, 1);
	face_colors[4] = pba::Color(1, 1, 0, 1);
	face_colors[5] = pba::Color(0, 1, 1, 1);

	std::vector<int> face;
	face.push_back(1);
	face.push_back(2);
	face.push_back(6);
	face.push_back(5);
	faces.push_back(face);

	face[0] = 2;
	face[1] = 3;
	face[2] = 7;
	face[3] = 6;
	faces.push_back(face);

	face[0] = 0;
	face[1] = 3;
	face[2] = 2;
	face[3] = 1;
	faces.push_back(face);

	face[0] = 0;
	face[1] = 4;
	face[2] = 7;
	face[3] = 3;
	faces.push_back(face);

	face[0] = 0;
	face[1] = 1;
	face[2] = 5;
	face[3] = 4;
	faces.push_back(face);

	face[0] = 5;
	face[1] = 6;
	face[2] = 7;
	face[3] = 4;
	faces.push_back(face);

	auto Vertex = [verts](const int i)
	{
		int ii = i % 8;
		pba::Vector result = verts[ii];
		return result;
	};

	CollisionSurface surf = makeCollisionSurface();

	for (size_t i = 0; i < faces.size(); i++)
	{
		std::vector<int>& face = faces[i];

		CollisionTriangle tri1 = makeCollisionTriangle(Vertex(face[0]), Vertex(face[1]), Vertex(face[2]));
		tri1->set_color(face_colors[i]);
		surf->addTriangle(tri1);

		CollisionTriangle tri2 = makeCollisionTriangle(Vertex(face[2]), Vertex(face[3]), Vertex(face[0]));
		tri2->set_color(face_colors[(i + 1) % 6]);
		surf->addTriangle(tri2);
	}

	return surf;
}

void pba::combineCollisionSurface(CollisionSurface& tar, CollisionSurface& s2)
{
	for (int i = 0; i < s2->triangle_size(); i++)
	{
		tar->addTriangle(s2->get_triangle(i));
	}
}

double pba::drand48()
{
	return (double)rand() / RAND_MAX;
}
