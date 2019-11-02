#include "OccupancyVolume.h"

pba::OccupancyVolume::OccupancyVolume(const AABB& aabb, const double h):
	bounds(aabb), cellsize(h)
{
	nx = ny = nz = 0;
}

void pba::OccupancyVolume::populate(const DynamicalStateData& pq)
{
	nx = abs((bounds.LLC().X() - bounds.URC().X()) / cellsize);
	ny = abs((bounds.LLC().Y() - bounds.URC().Y()) / cellsize);
	nz = abs((bounds.LLC().Z() - bounds.URC().Z()) / cellsize);

	contents.clear();
	total_mass.clear();
	contents.reserve(nx + ny + nz);

	auto isPointInsideAABB = [](const Vector& _llc, const Vector& _urc, const Vector& point)
	{
		pba::Vector min = _llc;
		pba::Vector max = _urc;
		return(point.X() >= min.X() && point.X() <= max.X() &&
			point.Y() >= min.Y() && point.Y() <= max.Y() &&
			point.Z() >= min.Z() && point.Z() <= max.Z());
	};


	for (size_t i = 0; i < pq.nb(); i++)
	{
		Vector pos = pq.pos(i);

		for (size_t x = 0; x < nx-1; x++){
			if ((pos.X() >= bounds.LLC().X()+ cellsize* x) && (pos.X() <= bounds.LLC().X() + cellsize * (x + 1))){
				for (size_t y = 0; y < ny-1; y++){
					if ((pos.Y() >= bounds.LLC().Y() + cellsize * y) && (pos.Y() <= bounds.LLC().Y() + cellsize * (y + 1))){
						for (size_t z = 0; z < nz - 1; z++){
							if ((pos.Z() >= bounds.LLC().Z() + cellsize * z) && (pos.Z() <= bounds.LLC().Z() + cellsize * (z + 1))){
								std::vector<size_t> test{x, y, z, i};
								bool isExistCoor = false;
								size_t tmpCellId;
								for (int ii = 0; ii < contents.size(); ii++)
								{
									if ((contents[ii][0] == test[0]) && (contents[ii][1] == test[1]) && (contents[ii][2] == test[2]))
									{
										isExistCoor = true;
										tmpCellId = ii;
									}
								}
								if (isExistCoor) // exist the i, j, k coordinate
								{
									contents[tmpCellId].push_back(i);
									total_mass[tmpCellId] = (total_mass[tmpCellId] + pq.mass(i));

// 									bool isExitPartic = false;
// 									for (int kk = 3; kk < contents[tmpCellId].size(); kk++)
// 									{
// 										if (contents[tmpCellId][kk] == i)
// 											isExitPartic = true;
// 									}
// 									if (!isExitPartic)
// 									{
// 										contents[tmpCellId].push_back(i);
// 										total_mass[tmpCellId] = (total_mass[tmpCellId] + pq.mass(test[i])) / contents[tmpCellId].size();
// 									}
								}
								else {
									contents.push_back(test);
									total_mass.push_back(pq.mass(i));
								}
							}
						}
					}
				}
			}
		}
	}
}

size_t pba::OccupancyVolume::cell_nb(size_t i, size_t j, size_t k) const
{
	for (auto cell : contents)
	{
		if ((cell[0] == i) && (cell[1] == j) && (cell[2] == k))
		{
			return (cell.size() - 3);
		}
	}
	return 0;
}

const std::vector<size_t>& pba::OccupancyVolume::cell_contents(size_t i, size_t j, size_t k) const
{
	size_t cell_id = index(i, j, k);
	std::vector<size_t>cell_contents = contents[cell_id];
	return cell_contents;
}

double pba::OccupancyVolume::cell_total_mass(size_t i, size_t j, size_t k) const
{
	size_t cell_id = index(i, j, k);
	return total_mass[cell_id];
}

void pba::OccupancyVolume::neighbor_cells(size_t i, size_t j, size_t k, std::vector<size_t>& neighbors) const
{
	neighbors.clear();
	size_t rang_i[2]{ i - 2, i + 2 };
	size_t rang_j[2]{ j - 2, j + 2 };
	size_t rang_k[2]{ k - 2, k + 2 };

	for (size_t ii = 0; ii < contents.size(); ii++)
	{
		if ((contents[ii][0] >= rang_i[0]) && (contents[ii][0] <= rang_i[1])&&
			(contents[ii][1] >= rang_j[0]) && (contents[ii][1] <= rang_j[1]) &&
			(contents[ii][2] >= rang_k[0]) && (contents[ii][2] <= rang_j[1]))
		{
			if (contents[ii].size() <= 3)
				return;

			neighbors.push_back(ii);

// 			for (int jj = 3; jj < contents[ii].size(); jj++)
// 			{
// 				neighbors.push_back(contents[ii][jj]);
// 			}
		}
	}
}

size_t pba::OccupancyVolume::index(size_t i, size_t j, size_t k) const
{
	for (size_t ii = 0; ii < contents.size(); ii++)
	{
		if ((contents[ii][0] == i) && (contents[ii][1] == j) && (contents[ii][2] == k))
		{
			return ii;
		}
	}
}

size_t pba::OccupancyVolume::index(const Vector& P) const
{
	for (size_t x = 0; x < nx - 1; x++) {
		if ((P.X() >= bounds.LLC().X() + cellsize * x) && (P.X() <= bounds.LLC().X() + cellsize * (x + 1))) {
			for (size_t y = 0; y < ny - 1; y++) {
				if ((P.Y() >= bounds.LLC().Y() + cellsize * y) && (P.Y() <= bounds.LLC().Y() + cellsize * (y + 1))) {
					for (size_t z = 0; z < nz - 1; z++) {
						if ((P.Z() >= bounds.LLC().Z() + cellsize * z) && (P.Z() <= bounds.LLC().Z() + cellsize * (z + 1))) {
							return index(x, y, z);
						}
					}
				}
			}
		}
	}
}

void pba::OccupancyVolume::set_cellsize(const double& v)
{
	cellsize = v;
}
