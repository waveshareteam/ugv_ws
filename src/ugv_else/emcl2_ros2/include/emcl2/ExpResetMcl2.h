// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__EXPRESETMCL2_H_
#define EMCL2__EXPRESETMCL2_H_

#include "emcl2/Mcl.h"

#include <memory>

namespace emcl2
{
class ExpResetMcl2 : public Mcl
{
      public:
	ExpResetMcl2(
	  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
	  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
	  double expansion_radius_position, double expansion_radius_orientation,
	  double extraction_rate, double successive_penetration_threshold, bool sensor_reset);
	~ExpResetMcl2();

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);

      private:
	double alpha_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	double extraction_rate_;
	double range_threshold_;
	bool sensor_reset_;

	void expansionReset(void);

	// bool Particle::isPenetrating(
	double nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan);
};

}  // namespace emcl2

#endif	// EMCL2__EXPRESETMCL2_H_
