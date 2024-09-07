// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "emcl2/Mcl.h"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <cmath>
#include <iostream>

namespace emcl2
{
Mcl::Mcl(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map)
: last_odom_(NULL), prev_odom_(NULL)
{
	odom_model_ = move(odom_model);
	map_ = move(map);
	scan_ = scan;

	if (num <= 0) {
		RCLCPP_ERROR(rclcpp::get_logger("emcl2_node"), "NO PARTICLE");
	}

	Particle particle(p.x_, p.y_, p.t_, 1.0 / num);
	for (int i = 0; i < num; i++) {
		particles_.push_back(particle);
	}

	processed_seq_ = -1;
	alpha_ = 1.0;

	for (int i = 0; i < (1 << 16); i++) {
		cos_[i] = cos(M_PI * i / (1 << 15));
		sin_[i] = sin(M_PI * i / (1 << 15));
	}
}

Mcl::~Mcl()
{
	delete last_odom_;
	delete prev_odom_;
}

void Mcl::resampling(void)
{
	std::vector<double> accum;
	accum.push_back(particles_[0].w_);
	for (size_t i = 1; i < particles_.size(); i++) {
		accum.push_back(accum.back() + particles_[i].w_);
	}

	std::vector<Particle> old(particles_);

	double start = static_cast<double>(rand()) / (RAND_MAX * particles_.size());
	double step = 1.0 / particles_.size();

	std::vector<int> chosen;

	size_t tick = 0;
	for (size_t i = 0; i < particles_.size(); i++) {
		while (accum[tick] <= start + i * step) {
			tick++;
			if (tick == particles_.size()) {
				RCLCPP_ERROR(rclcpp::get_logger("emcl2_node"), "RESAMPLING FAILED");
				exit(1);
			}
		}
		chosen.push_back(tick);
	}

	for (size_t i = 0; i < particles_.size(); i++) {
		particles_[i] = old[chosen[i]];
	}
}

void Mcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	if (processed_seq_ == scan_.seq_) {
		return;
	}

	Scan scan;
	int seq = -1;
	while (seq != scan_.seq_) {  // trying to copy the latest scan before next
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if (!inv) {
		for ([[maybe_unused]] auto & _ : scan.ranges_) {
			scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
			  scan.angle_min_ + (i++) * scan.angle_increment_));
		}
	} else {
		for ([[maybe_unused]] auto & _ : scan.ranges_) {
			scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
			  scan.angle_max_ - (i++) * scan.angle_increment_));
		}
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if (valid_beams == 0) {
		return;
	}

	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}

	if (normalizeBelief() > 0.000001) {
		resampling();
	} else {
		resetWeight();
	}

	processed_seq_ = scan_.seq_;
}

void Mcl::motionUpdate(double x, double y, double t)
{
	if (last_odom_ == NULL) {
		last_odom_ = new Pose(x, y, t);
		prev_odom_ = new Pose(x, y, t);
		return;
	} else {
		last_odom_->set(x, y, t);
	}

	Pose d = *last_odom_ - *prev_odom_;
	if (d.nearlyZero()) {
		return;
	}

	double fw_length = sqrt(d.x_ * d.x_ + d.y_ * d.y_);
	double fw_direction = atan2(d.y_, d.x_) - prev_odom_->t_;

	odom_model_->setDev(fw_length, d.t_);

	for (auto & p : particles_) {
		p.p_.move(
		  fw_length, fw_direction, d.t_, odom_model_->drawFwNoise(),
		  odom_model_->drawRotNoise());
	}

	prev_odom_->set(*last_odom_);
}

void Mcl::meanPose(
  double & x_mean, double & y_mean, double & t_mean, double & x_dev, double & y_dev, double & t_dev,
  double & xy_cov, double & yt_cov, double & tx_cov)
{
	double x, y, t, t2;
	x = y = t = t2 = 0.0;
	for (const auto & p : particles_) {
		x += p.p_.x_;
		y += p.p_.y_;
		t += p.p_.t_;
		t2 += normalizeAngle(p.p_.t_ + M_PI);
	}

	x_mean = x / particles_.size();
	y_mean = y / particles_.size();
	t_mean = t / particles_.size();
	double t2_mean = t2 / particles_.size();

	double xx, yy, tt, tt2;
	xx = yy = tt = tt2 = 0.0;
	for (const auto & p : particles_) {
		xx += pow(p.p_.x_ - x_mean, 2);
		yy += pow(p.p_.y_ - y_mean, 2);
		tt += pow(p.p_.t_ - t_mean, 2);
		tt2 += pow(normalizeAngle(p.p_.t_ + M_PI) - t2_mean, 2);
	}

	if (tt > tt2) {
		tt = tt2;
		t_mean = normalizeAngle(t2_mean - M_PI);
	}

	x_dev = xx / (particles_.size() - 1);
	y_dev = yy / (particles_.size() - 1);
	t_dev = tt / (particles_.size() - 1);

	double xy, yt, tx;
	xy = yt = tx = 0.0;
	for (const auto & p : particles_) {
		xy += (p.p_.x_ - x_mean) * (p.p_.y_ - y_mean);
		yt += (p.p_.y_ - y_mean) * (normalizeAngle(p.p_.t_ - t_mean));
		tx += (p.p_.x_ - x_mean) * (normalizeAngle(p.p_.t_ - t_mean));
	}

	xy_cov = xy / (particles_.size() - 1);
	yt_cov = yt / (particles_.size() - 1);
	tx_cov = tx / (particles_.size() - 1);
}

double Mcl::normalizeAngle(double t)
{
	while (t > M_PI) {
		t -= 2 * M_PI;
	}
	while (t < -M_PI) {
		t += 2 * M_PI;
	}

	return t;
}

void Mcl::setScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	if (msg->ranges.size() != scan_.ranges_.size()) {
		scan_.ranges_.resize(msg->ranges.size());
	}

	scan_.seq_ = msg->header.stamp.sec;
	for (size_t i = 0; i < msg->ranges.size(); i++) {
		scan_.ranges_[i] = msg->ranges[i];
	}

	scan_.angle_min_ = msg->angle_min;
	scan_.angle_max_ = msg->angle_max;
	scan_.angle_increment_ = msg->angle_increment;
	scan_.range_min_ = msg->range_min;
	scan_.range_max_ = msg->range_max;
}

double Mcl::normalizeBelief(void)
{
	double sum = 0.0;
	for (const auto & p : particles_) {
		sum += p.w_;
	}

	if (sum < 0.000000000001) {
		return sum;
	}

	for (auto & p : particles_) {
		p.w_ /= sum;
	}

	return sum;
}

void Mcl::resetWeight(void)
{
	for (auto & p : particles_) {
		p.w_ = 1.0 / particles_.size();
	}
}

void Mcl::initialize(double x, double y, double t)
{
	Pose new_pose(x, y, t);
	for (auto & p : particles_) {
		p.p_ = new_pose;
	}

	resetWeight();
}

void Mcl::simpleReset(void)
{
	std::vector<Pose> poses;
	map_->drawFreePoses(particles_.size(), poses);

	for (size_t i = 0; i < poses.size(); i++) {
		particles_[i].p_ = poses[i];
		particles_[i].w_ = 1.0 / particles_.size();
	}
}

double Mcl::cos_[(1 << 16)];
double Mcl::sin_[(1 << 16)];

}  // namespace emcl2
