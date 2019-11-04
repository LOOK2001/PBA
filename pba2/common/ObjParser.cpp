#include "ObjParser.h"
#include "PbaUtils.h"

const bool pba::ObjParser::ParseFile(const string& filename)
{
	mesh = meshData();
	parser.ParseFile(filename, mesh);

	return false;
}

const bool pba::ObjParser::Fill(CollisionSurface& surf)
{
	// vertices
	const std::vector<Vector>& verts = mesh.Positions;

	// faces
	const std::vector<unsigned int>& faces = mesh.Indices;

	// face colors
	pba::Color face_colors[6];

	face_colors[0] = pba::Color(1, 0, 1, 1);
	face_colors[1] = pba::Color(1, 0, 0, 1);
	face_colors[2] = pba::Color(0, 0, 1, 1);
	face_colors[3] = pba::Color(0, 1, 0, 1);
	face_colors[4] = pba::Color(1, 1, 0, 1);
	face_colors[5] = pba::Color(0, 1, 1, 1);

	
	for (size_t i = 0, j = 0; i < faces.size()-3;)
	{
		float size = 1.0;

		CollisionTriangle tri = makeCollisionTriangle(verts[faces[i]]* size, verts[faces[i+1]]* size, verts[faces[i+2]]* size);
		i += 3;
		tri->set_color(face_colors[(++j) % 6]);
		surf->addTriangle(tri);
	}

	return true;
}

const bool pba::ObjParser::Fill(RigidBodyState& g)
{
	DynamicalState ss = std::dynamic_pointer_cast<DynamicalStateData, RigidBodyStateData>(g);
	return Fill(ss);
}

const bool pba::ObjParser::Fill(DynamicalState& g)
{
	// vertices filter
	int verFilter = 1;

	// vertices
	const std::vector<Vector>& verts = mesh.Positions;
	Vector zero(0, 0, 0);

	if (!g)
		return false;

	size_t size = 0;
	for (size_t i = 0; i < verts.size(); i++)
	{
		if (!(i % verFilter == 0))
			continue;
		size++;
	}

	g->add(size);
	size_t i;
	Vector highest = verts[0];
	size_t index = 0;
	for (size_t i = 0; i < size; i++)
	{
		Color C(drand48(), drand48(), drand48(), 1.0);
		g->set_pos(i, verts[i* verFilter]);
		g->set_ci(i, C);
		g->set_vel(i, zero);
		g->set_mass(i, 1);

		if (verts[i * verFilter].Y() > highest.Y())
		{
			highest = verts[i * verFilter];
			index = i;
		}
	}

	g->set_mass(index, 1);

	return true;
}
