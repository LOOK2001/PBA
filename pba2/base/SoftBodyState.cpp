#include "SoftBodyState.h"
#include <cmath>


void pba::SoftBodyStateData::add_pair(size_t i, size_t j)
{
	double length = (pos(i) - pos(j)).magnitude();
	SoftEdge edge = CreateSoftEdge(i, j, length);
	connected_pairs.push_back(edge);
}

void pba::SoftBodyStateData::add_triangle(size_t i, size_t j, size_t k)
{
	double A = ((pos(j) - pos(i)) ^ (pos(k) - pos(i))).magnitude();
	SoftTriangle tri= CreateSoftTriangle(i, j, k, A);
	area_sets.push_back(tri);
}

void pba::SoftBodyStateData::add_bend(size_t i, size_t j, size_t k, size_t l)
{
	Vector e1 = pos(j) - pos(i);
	Vector e2 = pos(k) - pos(i);
	Vector n0 = (e1 ^ e2).magnitude();
	Vector f1 = pos(j) - pos(l);
	Vector f2 = pos(k) - pos(l);
	Vector n1 = (f2 - f1).magnitude();
	Vector h = (pos(k) - pos(j)).magnitude();

	double sint = h * (n0 ^ n1);
	double cost = n1 * n0;
	
	double theta = atan2(sint, cost);

	SoftBendable sb = CreateSoftBendable(i, j, k, l, theta);
	bend_pairs.push_back(sb);
}

pba::SoftBodyState pba::CreateSoftBody(const std::string& nam /*= "SoftBodyDataNoName"*/)
{
	return SoftBodyState(new SoftBodyStateData(nam));
}

pba::SoftBodyState pba::GeneratePlanarSoftBody(const Vector& llc, const Vector& urc, int nx, int nz)
{
	SoftBodyState sb = CreateSoftBody();

	sb->add((nx+1) * (nz+1));
	Vector vert;
	double dx = (urc.X() - llc.X()) / nx;
	double dz = (urc.Z() - llc.Z()) / nz;

	double edge_threshold = (dx + dz) * 0.5;
	sb->set_edge_threshold(edge_threshold);

	for (int i = 0; i <= nz; i++) {
		for (int j = 0; j <= nx; j++){
			vert = Vector(llc.X() + dx * i, llc.Y(), llc.Z() + dz * j);
			sb->set_pos(j + i * (nx + 1), vert);
		}
	}

	// CreateConnectedPairs
	for (int i = 0; i <= nz; ++i){ // z
		for (int j = 0; j <= nx; ++j){ // x
			size_t id = j + i * (nx + 1);
			// create connected pairs
			// add horizontal and vertical pairs
			if (j != nx)
				sb->add_pair(id, id + 1);
			if (i != nz)
				sb->add_pair(id, id + nx + 1);
		}
	}

	// CreateTriangleAreas
	for (int i = 0; i <= nz; ++i) { // z
		for (int j = 0; j <= nx; ++j) { // x
			size_t id = j + i * (nx + 1);
			// create triangle areas
			if (i != nz && j != nx)
			{
				// left upper
				sb->add_triangle(id, id + 1, id + nx + 1);
				// right upper
				sb->add_triangle(id, id + 1, id + nx + 2);
				// left lower
				sb->add_triangle(id, id + nx + 1, id + nx + 2);
				// right lower
				sb->add_triangle(id + 1, id + nx + 1, id + nx + 2);
			}
		}
	}

	// CreateBend
	for (int i = 0; i <= nz; ++i){ // z
		for (int j = 0; j <= nx; ++j) { // x
			size_t id = j + i * (nx + 1);
			// create bend
			if (i != nz && j != nx){
				sb->add_bend(id + nx + 1, id, id + nx + 2, id + 1);
			}
		}
	}

	return sb;
}