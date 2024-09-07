// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ExpResetMcl2.h"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <cmath>
#include <iostream>

namespace emcl2
{
ExpResetMcl2::ExpResetMcl2(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
  double expansion_radius_position, double expansion_radius_orientation, double extraction_rate,
  double range_threshold, bool sensor_reset)
: Mcl::Mcl(p, num, scan, odom_model, map),
  alpha_threshold_(alpha_th),
  expansion_radius_position_(expansion_radius_position),
  expansion_radius_orientation_(expansion_radius_orientation),
  extraction_rate_(extraction_rate),
  range_threshold_(range_threshold),
  sensor_reset_(sensor_reset)
{
}

ExpResetMcl2::~ExpResetMcl2() {}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	Scan scan;
	scan = scan_;

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	double origin = inv ? scan.angle_max_ : scan.angle_min_;
	int sgn = inv ? -1 : 1;
	for(size_t i = 0; i < scan.ranges_.size() ; i++) {
		scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
			origin + sgn * i * scan.angle_increment_));
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if (valid_beams == 0) {
		return;
	}

	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}

	alpha_ = nonPenetrationRate(static_cast<int>(particles_.size() * extraction_rate_), map_.get(), scan);
	RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "ALPHA: %f / %f", alpha_, alpha_threshold_);
	if (alpha_ < alpha_threshold_) {
		RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "RESET");
		expansionReset();
		for (auto & p : particles_) {
			p.w_ *= p.likelihood(map_.get(), scan);
		}
	}

	if (normalizeBelief() > 0.000001) {
		resampling();
	} else {
		resetWeight();
	}

	processed_seq_ = scan_.seq_;
}

double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for (size_t i = shift % skip; i < particles_.size(); i += skip) {
		counter++;
		if (particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_)) {
			penetrating++;
		}
	}
	shift++;

	std::cout << penetrating << " " << counter << std::endl;
	return static_cast<double>((counter - penetrating)) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
	for (auto & p : particles_) {
		double length =
		  2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * expansion_radius_position_;
		double direction = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;

		p.p_.x_ += length * cos(direction);
		p.p_.y_ += length * sin(direction);
		p.p_.t_ += 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) *
			   expansion_radius_orientation_;
		p.w_ = 1.0 / particles_.size();
	}
}

}  // namespace emcl2
