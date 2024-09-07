// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__ODOMMODEL_H_
#define EMCL2__ODOMMODEL_H_

#include <random>

namespace emcl2
{
class OdomModel
{
      public:
	OdomModel(double ff, double fr, double rf, double rr);
	void setDev(double length, double angle);
	double drawFwNoise(void);
	double drawRotNoise(void);

      private:
	double fw_dev_;
	double rot_dev_;

	double fw_var_per_fw_;
	double fw_var_per_rot_;
	double rot_var_per_fw_;
	double rot_var_per_rot_;

	std::random_device seed_gen_;
	std::default_random_engine engine_{seed_gen_()};

	std::normal_distribution<> std_norm_dist_;
};

}  // namespace emcl2

#endif	// EMCL2__ODOMMODEL_H_
