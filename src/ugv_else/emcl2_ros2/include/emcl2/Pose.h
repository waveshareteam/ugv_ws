// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__POSE_H_
#define EMCL2__POSE_H_

#include <sstream>
#include <string>

namespace emcl2
{

class Pose
{
      public:
	Pose() {}
	Pose(double x, double y, double t);
	Pose(const Pose & other);

	void set(double x, double y, double t);
	void set(const Pose & p);
	std::string to_s(void);

	void normalizeAngle(void);
	void move(
	  double length, double direction, double rotation, double fw_noise, double rot_noise);

	Pose operator-(const Pose & p) const;
	Pose operator=(const Pose & p);

	bool nearlyZero(void);

	double x_, y_, t_;

	uint16_t get16bitRepresentation(void);
	static uint16_t get16bitRepresentation(double);
};

}  // namespace emcl2

#endif	// EMCL2__POSE_H_
