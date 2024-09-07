// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__SCAN_H_
#define EMCL2__SCAN_H_

#include <iostream>
#include <vector>

namespace emcl2
{

class Scan
{
      public:
	int seq_;
	int scan_increment_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	double range_max_;
	double range_min_;

	double lidar_pose_x_;
	double lidar_pose_y_;
	double lidar_pose_yaw_;

	std::vector<double> ranges_;
	std::vector<uint16_t> directions_16bit_;

	Scan & operator=(const Scan & s);
	int countValidBeams(double * rate = NULL);
	bool valid(double range);
};

}  // namespace emcl2

#endif	// EMCL2__SCAN_H_
