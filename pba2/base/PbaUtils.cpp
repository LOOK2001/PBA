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

std::vector<pba::AABB> testAABB;

void pba::AddCollisionSurface(CollisionSurface& s, PbaThing& p)
{
	return;
}

void pba::Display(CollisionSurface& s)
{
	if (s->use_wireframe())
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < s->triangle_size(); i++)
	{
		if (!s->get_triangle(i)->visibility())
			continue;

		pba::Color color = s->get_triangle(i)->get_color();
		glColor4f(color.red(), color.green(), color.blue(), color.alpha());

		pba::Vector v = s->get_triangle(i)->vertex(0);
		glVertex3f(v.X(), v.Y(), v.Z());
		//glNormal3f(v.X(), v.Y(), v.Z());

		v = s->get_triangle(i)->vertex(1);
		glVertex3f(v.X(), v.Y(), v.Z());
		//glNormal3f(v.X(), v.Y(), v.Z());

		v = s->get_triangle(i)->vertex(2);
		glVertex3f(v.X(), v.Y(), v.Z());
		//glNormal3f(v.X(), v.Y(), v.Z());

		s->get_triangle(i)->decay();
	}

	if (!testAABB.empty())
	{
		for (auto box : testAABB)
		{
			glBegin(GL_TRIANGLES);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			pba::Vector	v1 = box.LLC();
			pba::Vector	v2 = box.URC();
			glVertex3f(v1.X(), v1.Y(), v1.Z());
			glVertex3f(v2.X(), v1.Y(), v1.Z());
			glVertex3f(v2.X(), v2.Y(), v1.Z());

			glVertex3f(v1.X(), v1.Y(), v1.Z());
			glVertex3f(v1.X(), v2.Y(), v1.Z());
			glVertex3f(v2.X(), v2.Y(), v1.Z());

			glVertex3f(v1.X(), v1.Y(), v1.Z());
			glVertex3f(v1.X(), v1.Y(), v2.Z());
			glVertex3f(v2.X(), v1.Y(), v1.Z());

			glVertex3f(v1.X(), v1.Y(), v2.Z());
			glVertex3f(v2.X(), v1.Y(), v2.Z());
			glVertex3f(v2.X(), v1.Y(), v1.Z());

			glPolygonMode(GL_FRONT_AND_BACK, GL_TRIANGLES);
		}
	}

	glEnd();
	return;
}

pba::CollisionSurface pba::GenerateCollisionCube(double size, const Vector& trans /*= Vector(0.0)*/)
{
	// vertices
	pba::Vector verts[8];

	// faces
	std::vector< std::vector<int> > faces;

	// face colors
	pba::Color face_colors[6];

	verts[0] = pba::Vector(-1 * size, -1 * size, -1 * size) + trans;
	verts[1] = pba::Vector(1 * size, -1 * size, -1 * size) + trans;
	verts[2] = pba::Vector(1 * size, 1 * size, -1 * size) + trans;
	verts[3] = pba::Vector(-1 * size, 1 * size, -1 * size) + trans;
	verts[4] = pba::Vector(-1 * size, -1 * size, 1 * size) + trans;
	verts[5] = pba::Vector(1 * size, -1 * size, 1 * size) + trans;
	verts[6] = pba::Vector(1 * size, 1 * size, 1 * size) + trans;
	verts[7] = pba::Vector(-1 * size, 1 * size, 1 * size) + trans;

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
		tri1->set_color(face_colors[i % 6]);
		surf->addTriangle(tri1);

		CollisionTriangle tri2 = makeCollisionTriangle(Vertex(face[2]), Vertex(face[3]), Vertex(face[0]));
		tri2->set_color(face_colors[(i + 1) % 6]);
		surf->addTriangle(tri2);
	}

	return surf;
}

pba::CollisionSurface pba::GenerateCollisionPlane(Vector coeff, const Matrix& _rot, const Vector& trans /*= Vector(0.0)*/)
{
	// vertices
	std::vector<pba::Vector> verts;
	verts.push_back(Vector(-1, -1, -1));
	verts.push_back(Vector(1, -1, -1));
	verts.push_back(Vector(-1, -1, 1));
	verts.push_back(Vector(1, -1, 1));

	// faces
	std::vector< std::vector<int> > faces;

	// face colors
	pba::Color face_colors[6];

	std::vector<glm::vec4> vec;
	vec.push_back(glm::vec4(-1, -1, -1, 1));
	vec.push_back(glm::vec4(1, -1, -1, 1));
	vec.push_back(glm::vec4(-1, -1, 1, 1));
	vec.push_back(glm::vec4(1, -1, 1, 1));

	for (auto v : verts)
	{
		v = v * _rot;
	}

	for (size_t i = 0; i < verts.size(); i++)
	{
		verts[i] = Vector(verts[i].X() * coeff.X(), verts[i].Y() * coeff.Y(), verts[i].Z() * coeff.Z());
	}

	face_colors[0] = pba::Color(1, 0, 1, 1);
	face_colors[1] = pba::Color(1, 0, 0, 1);
	face_colors[2] = pba::Color(0, 0, 1, 1);
	face_colors[3] = pba::Color(0, 1, 0, 1);
	face_colors[4] = pba::Color(1, 1, 0, 1);
	face_colors[5] = pba::Color(0, 1, 1, 1);

	std::vector<int> face;
	face.push_back(0);
	face.push_back(1);
	face.push_back(2);
	faces.push_back(face);

	face[0] = 2;
	face[1] = 3;
	face[2] = 1;
	faces.push_back(face);

	CollisionSurface surf = makeCollisionSurface();

	for (size_t i = 0; i < faces.size(); i++)
	{
		std::vector<int>& face = faces[i];

		CollisionTriangle tri1 = makeCollisionTriangle(verts[face[0]], verts[face[1]], verts[face[2]]);
		tri1->set_color(face_colors[i % 6]);
		surf->addTriangle(tri1);
	}

	return surf;
}

pba::CollisionSurface pba::GenerateCollisionSphere(double coeff, const Vector& trans /*= Vector(0.0)*/)
{
	std::vector<pba::Vector> verts;
	verts.resize(8);

	verts[0] = pba::Vector(-3, -3, -3);
	verts[1] = pba::Vector(3, -3, -3);
	verts[2] = pba::Vector(3, 3, -3);
	verts[3] = pba::Vector(-3, 3, -3);
	verts[4] = pba::Vector(-3, -3, 3);
	verts[5] = pba::Vector(3, -3, 3);
	verts[6] = pba::Vector(3, 3, 3);
	verts[7] = pba::Vector(-3, 3, 3);



	pba::Color face_colors[12];
	face_colors[0] = pba::Color(1, 0, 1, 0);
	face_colors[1] = pba::Color(1, 0, 0, 0);
	face_colors[2] = pba::Color(0, 0, 1, 0);
	face_colors[3] = pba::Color(0, 1, 0, 0);
	face_colors[4] = pba::Color(1, 1, 0, 0);
	face_colors[5] = pba::Color(0, 1, 1, 0);
	face_colors[6] = pba::Color(1, 0.5, 1, 0);
	face_colors[7] = pba::Color(1, 0.5, 0.5, 0);
	face_colors[8] = pba::Color(0.5, 0.5, 1, 0);
	face_colors[9] = pba::Color(0.5, 1, 0.5, 0);
	face_colors[10] = pba::Color(1, 1, 0.5, 0);
	face_colors[11] = pba::Color(0.5, 1, 1, 0);

	CollisionSurface box = makeCollisionSurface();

	CollisionTriangle tri = makeCollisionTriangle(verts[0], verts[1], verts[2]);
	tri->set_color(face_colors[0]);
	tri->set_invisible();
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[0], verts[2], verts[3]);
	tri->set_color(face_colors[1]);
	tri->set_invisible();
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[1], verts[5], verts[6]);
	tri->set_color(face_colors[2]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[1], verts[6], verts[2]);
	tri->set_color(face_colors[3]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[1], verts[5], verts[4]);
	tri->set_color(face_colors[4]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[1], verts[4], verts[0]);
	tri->set_color(face_colors[5]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[2], verts[6], verts[7]);
	tri->set_color(face_colors[6]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[2], verts[7], verts[3]);
	tri->set_color(face_colors[7]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[0], verts[4], verts[7]);
	tri->set_color(face_colors[8]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[0], verts[7], verts[3]);
	tri->set_color(face_colors[9]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[5], verts[6], verts[7]);
	tri->set_color(face_colors[10]);
	box->addTriangle(tri);

	tri = makeCollisionTriangle(verts[5], verts[7], verts[4]);
	tri->set_color(face_colors[11]);
	box->addTriangle(tri);

	return box;
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
