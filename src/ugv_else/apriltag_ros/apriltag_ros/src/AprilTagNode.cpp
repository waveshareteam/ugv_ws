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

#include <AprilTagNode.hpp>
// C Headers
#include <cv_bridge/cv_bridge.h>

// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

// C++ Headers
#include <image_transport/image_transport.hpp>

// create and delete functions for default tags
#define TAG_CREATE(name) {#name, tag ## name ## _create},
#define TAG_DESTROY(name) {#name, tag ## name ## _destroy},

const std::map<std::string, apriltag_family_t * (*)(void)> AprilTagNode::tag_create = {
  TAG_CREATE(36h11) TAG_CREATE(25h9) TAG_CREATE(16h5) TAG_CREATE(Circle21h7) TAG_CREATE(Circle49h12)
  TAG_CREATE(Custom48h12) TAG_CREATE(Standard41h12) TAG_CREATE(Standard52h13)};

const std::map<std::string, void (*)(apriltag_family_t *)> AprilTagNode::tag_destroy = {
  TAG_DESTROY(36h11) TAG_DESTROY(25h9) TAG_DESTROY(16h5) TAG_DESTROY(Circle21h7)
  TAG_DESTROY(Circle49h12) TAG_DESTROY(Custom48h12) TAG_DESTROY(Standard41h12)
  TAG_DESTROY(Standard52h13)};

AprilTagNode::AprilTagNode(rclcpp::NodeOptions options)
: Node("apriltag", options.use_intra_process_comms(true)),
  // parameter
  td(apriltag_detector_create()),
  tag_family(declare_parameter<std::string>("family", "36h11")),
  tag_edge_size(declare_parameter<double>("size", 2.0)),
  max_hamming(declare_parameter<int>("max_hamming", 0)),
  z_up(declare_parameter<bool>("z_up", true)),

  // topics
  sub_cam(image_transport::create_camera_subscription(
      this, "image",
      std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      declare_parameter<std::string>("image_transport", "raw"), rmw_qos_profile_default)),
  pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
      "apriltag_detections", rclcpp::QoS(10)))
{
  td->quad_decimate = declare_parameter<float>("decimate", 1.0);
  td->quad_sigma = declare_parameter<float>("blur", 0.0);
  td->nthreads = declare_parameter<int>("threads", 4);
  td->debug = declare_parameter<int>("debug", false);
  td->refine_edges = declare_parameter<int>("refine-edges", true);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this, rclcpp::QoS(10));
  // get tag names, IDs and sizes
  const auto ids = declare_parameter<std::vector<int64_t>>("tag_ids", {});
  const auto frames = declare_parameter<std::vector<std::string>>("tag_frames", {});

  if (!frames.empty()) {
    if (ids.size() != frames.size()) {
      throw std::runtime_error(
              "Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" +
              std::to_string(frames.size()) + ") mismatch!");
    }
    for (size_t i = 0; i < ids.size(); i++) {
      tag_frames[ids[i]] = frames[i];
    }
  }
  const auto sizes = declare_parameter<std::vector<double>>("tag_sizes", {});
  if (!sizes.empty()) {
    // use tag specific size
    if (ids.size() != sizes.size()) {
      throw std::runtime_error(
              "Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" +
              std::to_string(sizes.size()) + ") mismatch!");
    }
    for (size_t i = 0; i < ids.size(); i++) {
      tag_sizes[ids[i]] = sizes[i];
    }
  }
  if (tag_create.count(tag_family)) {
    tf = tag_create.at(tag_family)();
    apriltag_detector_add_family(td, tf);
  } else {
    throw std::runtime_error("Unsupported tag family: " + tag_family);
  }
}

AprilTagNode::~AprilTagNode()
{
  apriltag_detector_destroy(td);
  tag_destroy.at(tag_family)(tf);
}

int AprilTagNode::idComparison(const void * first, const void * second)
{
  int id1 = (reinterpret_cast<const apriltag_detection_t *>(first))->id;
  int id2 = (reinterpret_cast<const apriltag_detection_t *>(second))->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void AprilTagNode::removeDuplicates(zarray_t * detections_)
{
  zarray_sort(detections_, &AprilTagNode::idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while (true) {
    if (count > zarray_size(detections_) - 1) {
      // The entire detection set was parsed
      return;
    }
    apriltag_detection_t * detection;
    zarray_get(detections_, count, &detection);
    int id_current = detection->id;
    // Default id_next value of -1 ensures that if the last detection
    // is a duplicated tag ID, it will get removed
    int id_next = -1;
    if (count < zarray_size(detections_) - 1) {
      zarray_get(detections_, count + 1, &detection);
      id_next = detection->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected)) {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_, count, shuffle);
      if (id_current != id_next) {
        RCLCPP_WARN(
          get_logger(),
          "Pruning tag ID %d because it "
          "appears more than once in the image.",
          id_current);
        duplicate_detected = false;  // Reset
      }
      continue;
    } else {
      count++;
    }
  }
}

void AprilTagNode::onCamera(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci)
{
  // convert to 8bit monochrome image
  const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

  image_u8_t im = {
    .width = img_uint8.cols,
    .height = img_uint8.rows,
    .stride = img_uint8.cols,
    .buf = img_uint8.data};
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(msg_ci);
  double fx = camera_model.fx();  // focal length in camera x-direction [px]
  double fy = camera_model.fy();  // focal length in camera y-direction [px]
  double cx = camera_model.cx();  // optical center x-coordinate [px]
  double cy = camera_model.cy();  // optical center y-coordinate [px]

  // detect tags
  zarray_t * detections = apriltag_detector_detect(td, &im);
  if (remove_duplicates_) {
    removeDuplicates(detections);
  }
  apriltag_msgs::msg::AprilTagDetectionArray tag_detection_array;
  std::vector<std::string> detection_names;
  tag_detection_array.header = msg_img->header;
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t * det;
    zarray_get(detections, i, &det);
    // ignore untracked tags
    if (!tag_frames.empty() && !tag_frames.count(det->id)) {
      continue;
    }

    // reject detections with more corrected bits than allowed
    if (det->hamming > max_hamming) {
      continue;
    }

    std::vector<cv::Point3d> standaloneTagObjectPoints;
    std::vector<cv::Point2d> standaloneTagImagePoints;
    double tag_half_size = (tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size) / 2;
    addObjectPoints(tag_half_size, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(det, standaloneTagImagePoints);
    Eigen::Matrix4d transform =
      getRelativeTransform(standaloneTagObjectPoints, standaloneTagImagePoints, fx, fy, cx, cy);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    geometry_msgs::msg::TransformStamped tag_pose =
      makeTagPose(transform, rot_quaternion, msg_img->header);
    // 3D orientation and position
    // set child frame name by generic tag name or configured tag name
    tag_pose.child_frame_id = tag_frames.count(det->id) ?
      tag_frames.at(det->id) :
      std::string(det->family->name) + ":" + std::to_string(det->id);
    tf_broadcaster_->sendTransform(tag_pose);

    // Publish Tag detection msg
    apriltag_msgs::msg::AprilTagDetection tag_detection;
    tag_detection.pose.pose.pose.position.x = transform(0, 3);
    tag_detection.pose.pose.pose.position.y = transform(1, 3);
    tag_detection.pose.pose.pose.position.z = transform(2, 3);
    tag_detection.pose.pose.pose.orientation = tag_pose.transform.rotation;
    tag_detection.id = det->id;
    tag_detection.size = tag_half_size * 2;
    tag_detection_array.detections.push_back(tag_detection);
  }
  pub_detections->publish(tag_detection_array);
  apriltag_detections_destroy(detections);
}
void AprilTagNode::addObjectPoints(
  double s, cv::Matx44d T_oi, std::vector<cv::Point3d> & objectPoints) const
{
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void AprilTagNode::addImagePoints(
  apriltag_detection_t * detection, std::vector<cv::Point2d> & imagePoints) const
{
  // Add to image point vector the tag corners in the image frame
  // Going counterclockwise starting from the bottom left corner
  double tag_x[4] = {-1, 1, 1, -1};
  double tag_y[4] = {1, 1, -1, -1};  // Negated because AprilTag tag local
                                     // frame has y-axis pointing DOWN
                                     // while we use the tag local frame
                                     // with y-axis pointing UP
  for (int i = 0; i < 4; i++) {
    // Homography projection taking tag local frame coordinates to image pixels
    double im_x, im_y;
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.push_back(cv::Point2d(im_x, im_y));
  }
}

Eigen::Matrix4d AprilTagNode::getRelativeTransform(
  std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints, double fx, double fy,
  double cx, double cy) const
{
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Vec4f distCoeffs(0, 0, 0, 0);  // distortion coefficients
  // TODO(H-HChen) Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;

  if (z_up) {
    wRo << R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), R(0, 0), R(0, 1), R(0, 2);
  } else {
    wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
  }

  Eigen::Matrix4d T;  // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;
  return T;
}

geometry_msgs::msg::TransformStamped AprilTagNode::makeTagPose(
  const Eigen::Matrix4d & transform, const Eigen::Quaternion<double> rot_quaternion,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TransformStamped tf_;
  tf_.header = header;
  //===== Position and orientation
  tf_.transform.translation.x = transform(0, 3);
  tf_.transform.translation.y = transform(1, 3);
  tf_.transform.translation.z = transform(2, 3);

  if (z_up) {
    tf_.transform.rotation.x = rot_quaternion.z();
    tf_.transform.rotation.y = rot_quaternion.x();
    tf_.transform.rotation.z = rot_quaternion.y();
  } else {
    tf_.transform.rotation.x = rot_quaternion.x();
    tf_.transform.rotation.y = rot_quaternion.y();
    tf_.transform.rotation.z = rot_quaternion.z();
  }

  tf_.transform.rotation.w = rot_quaternion.w();
  return tf_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto tag_node = std::make_shared<AprilTagNode>();
  rclcpp::spin(tag_node);
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
