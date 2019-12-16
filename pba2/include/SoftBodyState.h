#ifndef __PBA_SOFTBODYSTATE_H__
#define __PBA_SOFTBODYSTATE_H__

#include "DynamicalState.h"
#include "SoftEdge.h"

namespace pba
{
	class SoftBodyStateData : public DynamicalStateData
	{
	public:
		SoftBodyStateData(const std::string& name = "SoftBodyDataNoName") {};
		SoftBodyStateData(const SoftBodyStateData& d) {};
		~SoftBodyStateData() {};

		SoftBodyStateData& operator= (const SoftBodyStateData& d);

		const SoftEdge& get_connected_pair(size_t p) const { return connected_pairs[p]; }
		const SoftTriangle& get_area_set(size_t p) const { return area_sets[p]; }
		const SoftBendable& get_bendable(size_t p) const { return bend_pairs[p]; }
		size_t nb_pairs() const { return connected_pairs.size(); }
		size_t nb_area_sets() const { return area_sets.size(); }
		size_t nb_bendables() const { return bend_pairs.size(); }
		bool empty() const { return connected_pairs.empty(); }

		void clear_pairs() { connected_pairs.clear(); area_sets.clear(); bend_pairs.clear(); }

		void add_pair(size_t i, size_t j);
		void add_triangle(size_t i, size_t j, size_t k);
		void add_bend(size_t i, size_t j, size_t k, size_t l);

		void set_edge_threshold(const double v) { edge_threshold = v; }
		double get_edge_threshold() const { return edge_threshold; }

	private:
		std::vector<SoftEdge> connected_pairs;
		double edge_threshold;

		std::vector<SoftTriangle> area_sets;	

		std::vector<SoftBendable> bend_pairs;
	};

	typedef std::shared_ptr<SoftBodyStateData> SoftBodyState;
	
	SoftBodyState CreateSoftBody(const std::string& nam = "SoftBodyDataNoName");

	SoftBodyState copy(const SoftBodyState d);

	SoftBodyState GeneratePlanarSoftBody(const pba::Vector& llc, const pba::Vector& urc, int nx, int ny, bool horz = true);
}

#endif // __PBA_SOFTBODYSTATE_H__
