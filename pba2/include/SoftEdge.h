#ifndef __PBA_SOFTEDGE_H__
#define __PBA_SOFTEDGE_H__

#include <memory>

namespace pba {
	class SoftEdgeData
	{
	public:
		SoftEdgeData(const size_t i, const size_t j, const double edgelength):
			inode(i),
			jnode(j),
			edge_length(edgelength)
		{}
		~SoftEdgeData() {}

		const size_t& get_first_node() const { return inode; }
		const size_t& get_second_node() const { return jnode; }
		const double get_edge_length() const { return edge_length; }

	private:
		size_t inode, jnode;
		double edge_length;
	};

	typedef std::shared_ptr<SoftEdgeData> SoftEdge;
	SoftEdge CreateSoftEdge(const size_t i, const size_t j, const double edgelength);


	class SoftTriangleData
	{
	public:
		SoftTriangleData(const size_t i, const size_t j, const size_t k, const double a) :
			inode(i),
			jnode(j),
			knode(k),
			area(a)
		{}
		~SoftTriangleData() {}

		const size_t& get_first_node() const { return inode; }
		const size_t& get_second_node() const { return jnode; }
		const size_t& get_third_node() const { return knode; }
		const double get_area() const { return area; }

	private:
		size_t inode, jnode, knode;
		double area;
	};

	typedef std::shared_ptr<SoftTriangleData> SoftTriangle;
	SoftTriangle CreateSoftTriangle(const size_t i, const size_t j, const size_t k, const double a);


	class SoftBendableData
	{
	public:
		SoftBendableData(const size_t i, const size_t j, const size_t k, const size_t l, const double a) :
			inode(i),
			jnode(j),
			knode(k),
			lnode(l),
			value(a)
		{}
		~SoftBendableData() {}

		const size_t& get_first_node() const { return inode; }
		const size_t& get_second_node() const { return jnode; }
		const size_t& get_third_node() const { return knode; }
		const size_t& get_fourth_node() const { return lnode; }
		const double get_value() const { return value; }

	private:
		size_t inode, jnode, knode, lnode;
		double value;
	};

	typedef std::shared_ptr<SoftBendableData> SoftBendable;
	SoftBendable CreateSoftBendable(const size_t i, const size_t j, const size_t k, const size_t l, const double a);
}

#endif
