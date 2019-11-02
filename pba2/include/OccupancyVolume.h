#ifndef __PBA_OCCUPANCY_VOLUME_H__
#define __PBA_OCCUPANCY_VOLUME_H__

#include "AABB.h"
#include "DynamicalState.h"

namespace pba
{
	class OccupancyVolume
	{
	public:
		OccupancyVolume(const AABB& aabb, const double h);
		OccupancyVolume(const OccupancyVolume& o);
		~OccupancyVolume() {};

		OccupancyVolume& operator=(const OccupancyVolume& o);

		void populate(const DynamicalStateData& pq);

		size_t nbx() const { return nx; };
		size_t nby() const { return ny; };;
		size_t nbz() const { return nz; };;
		size_t cell_nb(size_t i, size_t j, size_t k) const;
		const std::vector<size_t>& cell_contents(size_t i, size_t j, size_t k) const;
		Vector cell_center_of_mass(size_t i, size_t j, size_t k) const;
		double cell_total_mass(size_t i, size_t j, size_t k) const;

		void neighbor_cells(size_t i, size_t j, size_t k, std::vector<size_t>& neighbors) const;
		void distant_cells(size_t i, size_t j, size_t k, std::vector<size_t>& neighbors) const;

		size_t index(const Vector& P) const;
		size_t index(size_t i, size_t j, size_t k) const;
		void anti_index(const size_t ind, size_t& i, size_t& j, size_t& k) const;

		const double& get_cellsize() const { return cellsize; }
		void set_cellsize(const double& v);

		const size_t contents_nb() const { return contents.size(); }
		const void get_id_from_contents(size_t i, std::vector<size_t>& ids) const { ids = contents[i]; }

	private:
		AABB bounds;
		double cellsize;
		size_t nx, ny, nz;
		std::vector<std::vector<size_t>> contents;
		std::vector<Vector> center_of_mass;
		std::vector<double> total_mass;
		std::vector<Vector> cellID;

		void compute_size();
	};
}

#endif
