#include "SoftEdge.h"

pba::SoftEdge pba::CreateSoftEdge(const size_t i, const size_t j, const double edgelength)
{
	return SoftEdge(new SoftEdgeData(i, j, edgelength));
}

pba::SoftTriangle pba::CreateSoftTriangle(const size_t i, const size_t j, const size_t k, const double a)
{
	return SoftTriangle(new SoftTriangleData(i, j, k, a));
}

pba::SoftBendable pba::CreateSoftBendable(const size_t i, const size_t j, const size_t k, const size_t l, const double a)
{
	return SoftBendable(new SoftBendableData(i, j, k, l, a));
}
