#include <autonomos_atlas_lib/Atlas.h>
#include <autonomos_atlas_lib/Boundary.h>
#include <autonomos_atlas_lib/Lane.h>
#include <autonomos_atlas_lib/Lanes.h>
#include <autonomos_atlas_lib/RoadSection.h>
#include <autonomos_atlas_lib/RoadSections.h>
#include <autonomos_atlas_lib/Spline.h>
#include <autonomos_atlas_lib/lanemarking/LaneMarking.h>
#include <autonomos_atlas_lib/lanetype/LaneType.h>
#include <autonomos_atlas_lib/Length.h>
#include <fub_local_structured_trajectory_planner/TrajectoryPlanner.h>


#include <gtest/gtest.h>
#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

using std::string;
using ecl::Array;
using ecl::CubicPolynomial;
using ecl::CubicSpline;
using namespace std;
using namespace autonomos::atlas;

namespace fub {
namespace local_planner {

class optimal_planner {
private:
	nav_msgs::Path subsample(CubicSpline cubic,float start_x, float end_x);
	CubicSpline calculateSpline(Position p_start,Position p_end, double car_direction, double road_direction_p_end);
	double evaluateSpline(CubicSpline cubic,float start_x, float end_x);
	double scaled_value(Position pos,Position pos_min);

public:
	optimal_planner() {
	}

	~optimal_planner() {
	}
	vector<nav_msgs::Path> createOptimalPlan(vector<structured_planner::EdgePtr> route,double obstacle_velocity);

};

}

}
