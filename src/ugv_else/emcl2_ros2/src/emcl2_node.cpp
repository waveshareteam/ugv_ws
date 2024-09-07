// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
// CAUTION: Some lines came from amcl (LGPL).

#include "emcl2/emcl2_node.h"

#include "emcl2/LikelihoodFieldMap.h"
#include "emcl2/OdomModel.h"
#include "emcl2/Pose.h"
#include "emcl2/Scan.h"

#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <type_traits>
#include <utility>

namespace emcl2
{
EMcl2Node::EMcl2Node()
: Node("emcl2_node"),
  ros_clock_(RCL_SYSTEM_TIME),
  init_pf_(false),
  init_request_(false),
  simple_reset_request_(false),
  scan_receive_(false),
  map_receive_(false)
{
	// declare ros parameters
	declareParameter();
	initCommunication();
}

EMcl2Node::~EMcl2Node() {}

void EMcl2Node::declareParameter()
{
	this->declare_parameter("global_frame_id", std::string("map"));
	this->declare_parameter("footprint_frame_id", std::string("base_footprint"));
	this->declare_parameter("odom_frame_id", std::string("odom"));
	this->declare_parameter("base_frame_id", std::string("base_link"));

	this->declare_parameter("odom_freq", 20);
	this->declare_parameter("transform_tolerance", 0.2);

	this->declare_parameter("laser_min_range", 0.0);
	this->declare_parameter("laser_max_range", 100000000.0);
	this->declare_parameter("scan_increment", 1);

	this->declare_parameter("initial_pose_x", 0.0);
	this->declare_parameter("initial_pose_y", 0.0);
	this->declare_parameter("initial_pose_a", 0.0);

	this->declare_parameter("num_particles", 500);
	this->declare_parameter("alpha_threshold", 0.5);
	this->declare_parameter("expansion_radius_position", 0.1);
	this->declare_parameter("expansion_radius_orientation", 0.2);
	this->declare_parameter("extraction_rate", 0.1);
	this->declare_parameter("range_threshold", 0.1);
	this->declare_parameter("sensor_reset", false);

	this->declare_parameter("odom_fw_dev_per_fw", 0.19);
	this->declare_parameter("odom_fw_dev_per_rot", 0.0001);
	this->declare_parameter("odom_rot_dev_per_fw", 0.13);
	this->declare_parameter("odom_rot_dev_per_rot", 0.2);

	this->declare_parameter("laser_likelihood_max_dist", 0.2);
}

void EMcl2Node::initCommunication(void)
{
	particlecloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 2);
	pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 2);
	alpha_pub_ = create_publisher<std_msgs::msg::Float32>("alpha", 2);

	laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
	  "scan", 2, std::bind(&EMcl2Node::cbScan, this, std::placeholders::_1));
	initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	  "initialpose", 2,
	  std::bind(&EMcl2Node::initialPoseReceived, this, std::placeholders::_1));
	map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
	  "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
	  std::bind(&EMcl2Node::receiveMap, this, std::placeholders::_1));

	global_loc_srv_ = create_service<std_srvs::srv::Empty>(
	  "global_localization",
	  std::bind(&EMcl2Node::cbSimpleReset, this, std::placeholders::_1, std::placeholders::_2));

	this->get_parameter("global_frame_id", global_frame_id_);
	this->get_parameter("footprint_frame_id", footprint_frame_id_);
	this->get_parameter("odom_frame_id", odom_frame_id_);
	this->get_parameter("base_frame_id", base_frame_id_);

	this->get_parameter("odom_freq", odom_freq_);

	this->get_parameter("transform_tolerance", transform_tolerance_);
}

void EMcl2Node::initTF(void)
{
	tfb_.reset();
	tfl_.reset();
	tf_.reset();

	tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	  get_node_base_interface(), get_node_timers_interface(),
	  create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
	tf_->setCreateTimerInterface(timer_interface);
	tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
	tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
	latest_tf_ = tf2::Transform::getIdentity();
}

void EMcl2Node::initPF(void)
{
	std::shared_ptr<LikelihoodFieldMap> map = std::move(initMap());
	std::shared_ptr<OdomModel> om = std::move(initOdometry());

	Scan scan;
	this->get_parameter("laser_min_range", scan.range_min_);
	this->get_parameter("laser_max_range", scan.range_max_);
	this->get_parameter("scan_increment", scan.scan_increment_);

	Pose init_pose;
	this->get_parameter("initial_pose_x", init_pose.x_);
	this->get_parameter("initial_pose_y", init_pose.y_);
	this->get_parameter("initial_pose_a", init_pose.t_);

	int num_particles;
	double alpha_th;
	double ex_rad_pos, ex_rad_ori;
	this->get_parameter("num_particles", num_particles);
	this->get_parameter("alpha_threshold", alpha_th);
	this->get_parameter("expansion_radius_position", ex_rad_pos);
	this->get_parameter("expansion_radius_orientation", ex_rad_ori);

	double extraction_rate, range_threshold;
	bool sensor_reset = false;
	this->get_parameter("extraction_rate", extraction_rate);
	this->get_parameter("range_threshold", range_threshold);
	this->get_parameter("sensor_reset", sensor_reset);

	pf_.reset(new ExpResetMcl2(
	  init_pose, num_particles, scan, om, map, alpha_th, ex_rad_pos, ex_rad_ori,
	  extraction_rate, range_threshold, sensor_reset));

	init_pf_ = true;
}

std::shared_ptr<OdomModel> EMcl2Node::initOdometry(void)
{
	double ff, fr, rf, rr;
	this->get_parameter("odom_fw_dev_per_fw", ff);
	this->get_parameter("odom_fw_dev_per_rot", fr);
	this->get_parameter("odom_rot_dev_per_fw", rf);
	this->get_parameter("odom_rot_dev_per_rot", rr);
	return std::shared_ptr<OdomModel>(new OdomModel(ff, fr, rf, rr));
}

std::shared_ptr<LikelihoodFieldMap> EMcl2Node::initMap(void)
{
	double likelihood_range;
	this->get_parameter("laser_likelihood_max_dist", likelihood_range);

	return std::shared_ptr<LikelihoodFieldMap>(new LikelihoodFieldMap(map_, likelihood_range));
}

void EMcl2Node::receiveMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
	map_ = *msg;
	map_receive_ = true;
	RCLCPP_INFO(get_logger(), "Received map.");
	initPF();
	initTF();
}

void EMcl2Node::cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	if (init_pf_) {
		scan_receive_ = true;
		scan_time_stamp_ = msg->header.stamp;
		scan_frame_id_ = msg->header.frame_id;
		pf_->setScan(msg);
	}
}

void EMcl2Node::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	RCLCPP_INFO(get_logger(), "Run receiveInitialPose");
	if (!initialpose_receive_) {
		if (scan_receive_ && map_receive_) {
			init_x_ = msg->pose.pose.position.x;
			init_y_ = msg->pose.pose.position.y;
			init_t_ = tf2::getYaw(msg->pose.pose.orientation);
			pf_->initialize(init_x_, init_y_, init_t_);
			initialpose_receive_ = true;
		} else {
			if (!scan_receive_) {
				RCLCPP_WARN(
				  get_logger(),
				  "Not yet received scan. Therefore, MCL cannot be initiated.");
			}
			if (!map_receive_) {
				RCLCPP_WARN(
				  get_logger(),
				  "Not yet received map. Therefore, MCL cannot be initiated.");
			}
		}
	} else {
		init_request_ = true;
		init_x_ = msg->pose.pose.position.x;
		init_y_ = msg->pose.pose.position.y;
		init_t_ = tf2::getYaw(msg->pose.pose.orientation);
	}
}

void EMcl2Node::loop(void)
{
	if (init_request_) {
		pf_->initialize(init_x_, init_y_, init_t_);
		init_request_ = false;
	} else if (simple_reset_request_) {
		pf_->simpleReset();
		simple_reset_request_ = false;
	}

	if (init_pf_) {
		double x, y, t;
		if (!getOdomPose(x, y, t)) {
			RCLCPP_INFO(get_logger(), "can't get odometry info");
			return;
		}
		pf_->motionUpdate(x, y, t);

		double lx, ly, lt;
		bool inv;
		if (!getLidarPose(lx, ly, lt, inv)) {
			RCLCPP_INFO(get_logger(), "can't get lidar pose info");
			return;
		}

		pf_->sensorUpdate(lx, ly, lt, inv);

		double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
		pf_->meanPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);

		publishOdomFrame(x, y, t);
		publishPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
		publishParticles();

		std_msgs::msg::Float32 alpha_msg;
		alpha_msg.data = static_cast<float>(pf_->alpha_);
		alpha_pub_->publish(alpha_msg);
	} else {
		if (!scan_receive_) {
			RCLCPP_WARN(
			  get_logger(),
			  "Not yet received scan. Therefore, MCL cannot be initiated.");
		}
		if (!map_receive_) {
			RCLCPP_WARN(
			  get_logger(),
			  "Not yet received map. Therefore, MCL cannot be initiated.");
		}
	}
}

void EMcl2Node::publishPose(
  double x, double y, double t, double x_dev, double y_dev, double t_dev, double xy_cov,
  double yt_cov, double tx_cov)
{
	geometry_msgs::msg::PoseWithCovarianceStamped p;
	p.header.frame_id = global_frame_id_;
	p.header.stamp = ros_clock_.now();
	p.pose.pose.position.x = x;
	p.pose.pose.position.y = y;
	p.pose.covariance[6 * 0 + 0] = x_dev;
	p.pose.covariance[6 * 1 + 1] = y_dev;
	p.pose.covariance[6 * 2 + 2] = t_dev;
	p.pose.covariance[6 * 0 + 1] = xy_cov;
	p.pose.covariance[6 * 1 + 0] = xy_cov;
	p.pose.covariance[6 * 0 + 2] = tx_cov;
	p.pose.covariance[6 * 2 + 0] = tx_cov;
	p.pose.covariance[6 * 1 + 2] = yt_cov;
	p.pose.covariance[6 * 2 + 1] = yt_cov;

	tf2::Quaternion q;
	q.setRPY(0, 0, t);
	tf2::convert(q, p.pose.pose.orientation);

	pose_pub_->publish(p);
}

void EMcl2Node::publishOdomFrame(double x, double y, double t)
{
	geometry_msgs::msg::PoseStamped odom_to_map;
	try {
		tf2::Quaternion q;
		q.setRPY(0, 0, t);
		tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));

		geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = footprint_frame_id_;
		tmp_tf_stamped.header.stamp = scan_time_stamp_;
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

		tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
		return;
	}
	tf2::convert(odom_to_map.pose, latest_tf_);
	auto stamp = tf2_ros::fromMsg(scan_time_stamp_);
	tf2::TimePoint transform_expiration = stamp + tf2::durationFromSec(transform_tolerance_);

	geometry_msgs::msg::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = global_frame_id_;
	tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
	tmp_tf_stamped.child_frame_id = odom_frame_id_;
	tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

	tfb_->sendTransform(tmp_tf_stamped);
}

void EMcl2Node::publishParticles(void)
{
	geometry_msgs::msg::PoseArray cloud_msg;
	cloud_msg.header.stamp = ros_clock_.now();
	cloud_msg.header.frame_id = global_frame_id_;
	cloud_msg.poses.resize(pf_->particles_.size());

	for (size_t i = 0; i < pf_->particles_.size(); i++) {
		cloud_msg.poses[i].position.x = pf_->particles_[i].p_.x_;
		cloud_msg.poses[i].position.y = pf_->particles_[i].p_.y_;
		cloud_msg.poses[i].position.z = 0;

		tf2::Quaternion q;
		q.setRPY(0, 0, pf_->particles_[i].p_.t_);
		tf2::convert(q, cloud_msg.poses[i].orientation);
	}
	particlecloud_pub_->publish(cloud_msg);
}

bool EMcl2Node::getOdomPose(double & x, double & y, double & yaw)
{
	geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = footprint_frame_id_;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped odom_pose;
	try {
		this->tf_->transform(ident, odom_pose, odom_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	x = odom_pose.pose.position.x;
	y = odom_pose.pose.position.y;
	yaw = tf2::getYaw(odom_pose.pose.orientation);

	return true;
}

bool EMcl2Node::getLidarPose(double & x, double & y, double & yaw, bool & inv)
{
	geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = scan_frame_id_;
	ident.header.stamp = ros_clock_.now();
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped lidar_pose;
	try {
		this->tf_->transform(ident, lidar_pose, base_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute lidar pose, skipping scan (%s)", e.what());
		return false;
	}

	x = lidar_pose.pose.position.x;
	y = lidar_pose.pose.position.y;

	double roll, pitch;
	tf2::getEulerYPR(lidar_pose.pose.orientation, yaw, pitch, roll);
	inv = (fabs(pitch) > M_PI / 2 || fabs(roll) > M_PI / 2) ? true : false;

	return true;
}

int EMcl2Node::getOdomFreq(void) { return odom_freq_; }

bool EMcl2Node::cbSimpleReset(
  const std_srvs::srv::Empty::Request::ConstSharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
	return simple_reset_request_ = true;
}

}  // namespace emcl2

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<emcl2::EMcl2Node>();
	rclcpp::Rate loop_rate(node->getOdomFreq());
	while (rclcpp::ok()) {
		node->loop();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
