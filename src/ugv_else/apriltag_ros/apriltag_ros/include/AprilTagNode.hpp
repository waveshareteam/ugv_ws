// Copyright (c) 2022 H-HChen
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

// C Headers
#include <apriltag.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>


// C++ Headers
#include <map>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// apriltag
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "common/homography.h"


class AprilTagNode : public rclcpp::Node
{
public:
  explicit AprilTagNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~AprilTagNode() override;

private:
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

  apriltag_family_t * tf;
  apriltag_detector_t * const td;
  std::string tag_family;
  double tag_edge_size;
  int max_hamming;
  std::unordered_map<int, std::string> tag_frames;
  std::unordered_map<int, double> tag_sizes;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool remove_duplicates_ = true;
  bool z_up;
  // function pointer for tag family creation / destruction
  static const std::map<std::string, apriltag_family_t * (*)(void)> tag_create;
  static const std::map<std::string, void (*)(apriltag_family_t *)> tag_destroy;

  image_transport::CameraSubscriber sub_cam;
  const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

  void onCamera(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci);

  void getPose(const matd_t & H, geometry_msgs::msg::Transform & t, const double size) const;
  void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d> & objectPoints) const;
  void addImagePoints(
    apriltag_detection_t * detection, std::vector<cv::Point2d> & imagePoints) const;
  Eigen::Matrix4d getRelativeTransform(
    std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints, double fx,
    double fy, double cx, double cy) const;

  geometry_msgs::msg::TransformStamped makeTagPose(
    const Eigen::Matrix4d & transform, const Eigen::Quaternion<double> rot_quaternion,
    const std_msgs::msg::Header & header);

  static int idComparison(const void * first, const void * second);
  void removeDuplicates(zarray_t * detections_);
};
