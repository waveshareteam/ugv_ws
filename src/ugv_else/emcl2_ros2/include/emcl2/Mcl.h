// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__MCL_H_
#define EMCL2__MCL_H_

#include "emcl2/LikelihoodFieldMap.h"
#include "emcl2/OdomModel.h"
#include "emcl2/Particle.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <random>
#include <sstream>
#include <vector>

namespace emcl2
{
class Mcl
{
      public:
	Mcl() {}
	Mcl(
	  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
	  const std::shared_ptr<LikelihoodFieldMap> & map);
	~Mcl();

	std::vector<Particle> particles_;
	double alpha_;

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);
	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void setScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	void meanPose(
	  double & x_mean, double & y_mean, double & t_mean, double & x_var, double & y_var,
	  double & t_var, double & xy_cov, double & yt_cov, double & tx_cov);

	void simpleReset(void);

	static double cos_[(1 << 16)];
	static double sin_[(1 << 16)];

      protected:
	Pose * last_odom_;
	Pose * prev_odom_;

	Scan scan_;
	int processed_seq_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalizeBelief(void);
	void resetWeight(void);

	std::shared_ptr<OdomModel> odom_model_;
	std::shared_ptr<LikelihoodFieldMap> map_;
};

}  // namespace emcl2

#endif	// EMCL2__MCL_H_
