#include "geometry2D.h"
#include "nn.h"
#include <memory>

class ICP {
public:
	ICP(std::vector<Point>& reference, const double& gate);

	void SetRef(std::vector<Point>& reference, const double& gate);
	
	// Align observe point-set to reference point-set
	Pose align(const std::vector<Point>& observe, const Pose& init, 
		double gate, int iterations, double convergeErr, bool interpolate=false);
	double compute_epsilon( std::vector<nrr_TrICP_Pack>& dataset );

	// Get associated points after alignment step
//	const std::vector<Point>& get_points_ref() { return b; }
//	const std::vector<Point>& get_points_obs() { return a; }

private:
	std::vector<Point>& ref;
	std::auto_ptr<SweepSearch> nn;

	std::vector<nrr_TrICP_Pack> TrICP_Pack;
	double eps;
	
	double before_E_dist;
	Pose before_pse;
	static bool di2_order (const nrr_TrICP_Pack& a, const nrr_TrICP_Pack& b)
	{ return a.di2 < b.di2; }
};

