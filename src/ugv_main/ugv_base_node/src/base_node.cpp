#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Variables for quaternion and yaw angles
float q1, q2, q3;
float q0 = 1.0;
float imu_yaw = 0.0;
float odom_yaw = 0.0;
float yaw = 0.0;

// Macro to calculate the number of elements in an array
#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Covariance matrices for the pose and twist in the odometry message
const double ODOM_POSE_COVARIANCE[] = {1e-3, 0, 0, 0, 0, 0,
                                       0, 1e-3, 0, 0, 0, 0,
                                       0, 0, 1e6, 0, 0, 0,
                                       0, 0, 0, 1e6, 0, 0,
                                       0, 0, 0, 0, 1e6, 0,
                                       0, 0, 0, 0, 0, 1e3};
const double ODOM_POSE_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0,
                                        0, 1e-3, 1e-9, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e-9};

const double ODOM_TWIST_COVARIANCE[] = {1e-3, 0, 0, 0, 0, 0,
                                        0, 1e-3, 0, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e3};
const double ODOM_TWIST_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0,
                                         0, 1e-3, 1e-9, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e-9};

// Class to publish odometry data and broadcast transformations
class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_raw_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_; // Timer to control publishing rate

    double dt = 0.0;
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    float pre_odl;
    float pre_odr;
    float vx;  // Linear velocity
    float vw;  // Angular velocity
    bool pub_odom_tf_ = false;
    bool is_initialized = false;
    rclcpp::Time last_time_;
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";
    float init_odl = 0.0; // Initial value for left wheel encoder
    float init_odr = 0.0; // Initial value for right wheel encoder

public:
    OdomPublisher()
        : Node("base_node")
    {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
        this->declare_parameter<bool>("pub_odom_tf", false);

        this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
        this->get_parameter<std::string>("odom_frame", odom_frame);
        this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame);

        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to IMU and raw odometry data topics
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 5, std::bind(&OdomPublisher::handle_imu, this, _1));
        odom_raw_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("odom/odom_raw", 50, std::bind(&OdomPublisher::handle_odom, this, _1));

        // Publisher for odometry messages
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

        // Timer to publish odometry data periodically
        timer_ = this->create_wall_timer(100ms, std::bind(&OdomPublisher::publish_odom, this));
    }

private:
    // Callback to handle IMU data and compute yaw angle from quaternion
    void handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
        q1 = msg->orientation.x;
        q2 = msg->orientation.y;
        q3 = msg->orientation.z;
        q0 = msg->orientation.w;

        // Calculate yaw angle from quaternion
        double siny_cosp = 2 * (q0 * q3 + q1 * q2);
        double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        imu_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // Callback to handle raw odometry data and update position/velocity
    void handle_odom(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg)
    {
        rclcpp::Time curren_time = rclcpp::Clock().now();

        float now_odl = msg->data.at(0);  // Left wheel odometry
        float now_odr = msg->data.at(1);  // Right wheel odometry

        // Initialize encoders if it's the first callback
        if (!is_initialized)
        {
            init_odl = now_odl;
            init_odr = now_odr;
            is_initialized = true;
        }

        // Adjust odometry readings by subtracting the initial values
        now_odl -= init_odl;
        now_odr -= init_odr;

        // Calculate time delta
        dt = (curren_time - last_time_).seconds();
        last_time_ = curren_time;

        // Compute distance traveled by each wheel
        float dleft = now_odl - pre_odl;
        float dright = now_odr - pre_odr;

        // Update previous encoder readings
        pre_odl = now_odl;
        pre_odr = now_odr;

        // Calculate average distance and change in heading
        float dxy_ave = (dright + dleft) / 2.0;
        float dth = (dright - dleft) / 0.175;

        // Compute linear and angular velocities
        vx = dxy_ave / dt;
        vw = dth / dt;

        // Update position if robot has moved
        if (dxy_ave != 0)
        {
            float dx = cos(dth / 2) * dxy_ave;
            float dy = sin(dth / 2) * dxy_ave;
            x_pos_ += (cos(yaw) * dx - sin(yaw) * dy);
            y_pos_ += (sin(yaw) * dx + cos(yaw) * dy);
        }

        // Update heading if the robot has rotated
        if (dth != 0)
        {
            odom_yaw += dth;
        }

        // Use IMU yaw if available
        yaw = imu_yaw != 0 ? imu_yaw : odom_yaw;
    }

    // Function to publish odometry data and broadcast transformation
    void publish_odom()
    {
        auto odom = nav_msgs::msg::Odometry();
        auto trans = geometry_msgs::msg::TransformStamped();

        // Set the header information
        odom.header.stamp = rclcpp::Clock().now();
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_footprint_frame;

        // Set the position and orientation in the odometry message
        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.orientation.x = q1;
        odom.pose.pose.orientation.y = q2;
        odom.pose.pose.orientation.z = q3;
        odom.pose.pose.orientation.w = q0;

        // Choose covariance matrix based on the robot's state
        if (vx == 0 && vw == 0)
        {
            std::copy(std::begin(ODOM_POSE_COVARIANCE2), std::end(ODOM_POSE_COVARIANCE2), std::begin(odom.pose.covariance));
            std::copy(std::begin(ODOM_TWIST_COVARIANCE2), std::end(ODOM_TWIST_COVARIANCE2), std::begin(odom.twist.covariance));
        }
        else
        {
            std::copy(std::begin(ODOM_POSE_COVARIANCE), std::end(ODOM_POSE_COVARIANCE), std::begin(odom.pose.covariance));
            std::copy(std::begin(ODOM_TWIST_COVARIANCE), std::end(ODOM_TWIST_COVARIANCE), std::begin(odom.twist.covariance));
        }

        // Set linear and angular velocities in the odometry message
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vw;

        // Publish the odometry message
        odom_publisher_->publish(odom);

        // If enabled, broadcast the transformation
        if (pub_odom_tf_)
        {
            trans.header.stamp = rclcpp::Clock().now();
            trans.header.frame_id = odom_frame;
            trans.child_frame_id = base_footprint_frame;

            // Set translation and rotation for the transform
            trans.transform.translation.x = x_pos_;
            trans.transform.translation.y = y_pos_;
            trans.transform.rotation.x = q1;
            trans.transform.rotation.y = q2;
            trans.transform.rotation.z = q3;
            trans.transform.rotation.w = q0;

            // Broadcast the transformation
            tf_broadcaster_->sendTransform(trans);
        }
    }
};

// Main function to initialize the node and spin
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
