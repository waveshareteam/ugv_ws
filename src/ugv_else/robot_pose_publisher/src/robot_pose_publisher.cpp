/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Milan - milan.madathiparambil@gmail.com
 * \date April 20 1020
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RobotPosePublisher : public rclcpp::Node
{
public:
	RobotPosePublisher() : Node("robot_pose_publisher")
	{
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		this->declare_parameter<std::string>("map_frame","map");
		this->declare_parameter<std::string>("base_frame","base_footprint");
		this->declare_parameter<bool>("is_stamped",false);

		this->get_parameter("map_frame", map_frame);
		this->get_parameter("base_frame", base_frame);
		this->get_parameter("is_stamped", is_stamped);

		if (is_stamped)
			publisher_stamp = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 1);
		else
			publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pose", 1);
		timer_ = this->create_wall_timer(
			50ms, std::bind(&RobotPosePublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		geometry_msgs::msg::TransformStamped transformStamped;
		try
		{
			transformStamped = tf_buffer_->lookupTransform(map_frame, base_frame,
														   this->now());
		}
		catch (tf2::TransformException &ex)
		{
			return;
		}
		geometry_msgs::msg::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = map_frame;
		pose_stamped.header.stamp = this->now();

		pose_stamped.pose.orientation.x = transformStamped.transform.rotation.x;
		pose_stamped.pose.orientation.y = transformStamped.transform.rotation.y;
		pose_stamped.pose.orientation.z = transformStamped.transform.rotation.z;
		pose_stamped.pose.orientation.w = transformStamped.transform.rotation.w;

		pose_stamped.pose.position.x = transformStamped.transform.translation.x;
		pose_stamped.pose.position.y = transformStamped.transform.translation.y;
		pose_stamped.pose.position.z = transformStamped.transform.translation.z;

		if (is_stamped)
			publisher_stamp->publish(pose_stamped);
		else
			publisher_->publish(pose_stamped.pose);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_stamp;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
	size_t count_;
	bool is_stamped = false;
	std::string base_frame = "base_footprint";
	std::string map_frame = "map";
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotPosePublisher>());
	rclcpp::shutdown();
	return 0;
}
