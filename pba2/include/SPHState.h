#ifndef __PBA_SPHSTATE_H__
#define __PBA_SPHSTATE_H__

#include "DynamicalState.h"
#include "OccupancyVolume.h"

namespace pba 
{
	class SPHStateData : public DynamicalStateData, public OccupancyVolume
	{
	public:
		SPHStateData(const AABB& aabb, const double h, const std::string& nam = "SPHDataNoName");
		//SPHStateData(const SPHStateData& d);
		~SPHStateData() {};

		SPHStateData& operator= (const SPHStateData& d);

		const float get_radius() const { return radius; }
		void set_radius(const float& v);

		const float weight(size_t p, const Vector& P) const;
		const Vector grad_weight(size_t p, const Vector& P) const;

		void compute_density();
		void populate();

	private:
		float radius;
	};

	typedef std::shared_ptr<SPHStateData> SPHState;
	SPHState CreateSPH(const AABB& bounds, const double h, const std::string& nam = "SPHDataNoName");
	SPHState copy(const SPHState d);
}

#endif // !__PBA_SPHSTATE_H__