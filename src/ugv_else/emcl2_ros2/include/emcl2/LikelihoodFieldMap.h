// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__LIKELIHOODFIELDMAP_H_
#define EMCL2__LIKELIHOODFIELDMAP_H_

#include "emcl2/Pose.h"
#include "emcl2/Scan.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <utility>
#include <vector>

namespace emcl2
{
class LikelihoodFieldMap
{
      public:
	LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid & map, double likelihood_range);
	~LikelihoodFieldMap();

	void setLikelihood(int x, int y, double range);
	uint8_t likelihood(double x, double y);

	std::vector<uint8_t *> likelihoods_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;

	void drawFreePoses(int num, std::vector<Pose> & result);

      private:
	std::vector<std::pair<int, int>> free_cells_;

	void normalize(void);
};

}  // namespace emcl2

#endif	// EMCL2__LIKELIHOODFIELDMAP_H_
