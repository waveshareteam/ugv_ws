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

// Declare variables
float q1, q2, q3;
float q0 = 1.0;
float imu_yaw = 0.0;
float odom_yaw = 0.0;
float yaw = 0.0;

#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Define covariance matrices for odometry
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

// Define class for odometry publisher
class OdomPublisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_raw_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_; 

    double dt = 0.0;
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    float pre_odl;
    float pre_odr;
    float vx;
    float vw;     
    bool pub_odom_tf_ = false;
    bool is_initialized = false;
    rclcpp::Time last_time_;
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";
    float init_odl = 0.0;
    float init_odr = 0.0; 

public:
    OdomPublisher()
        : Node("base_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
        this->declare_parameter<bool>("pub_odom_tf", false);

        // Get parameters
        this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
        this->get_parameter<std::string>("odom_frame", odom_frame);
        this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create subscriptions
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 5, std::bind(&OdomPublisher::handle_imu, this, _1));
        odom_raw_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("odom/odom_raw", 50, std::bind(&OdomPublisher::handle_odom, this, _1));

        // Create publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 5);

        // Create timer
        timer_ = this->create_wall_timer(100ms, std::bind(&OdomPublisher::publish_odom, this));
    }

private:
    // Handle IMU data
    void handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
        q1 = msg->orientation.x;
        q2 = msg->orientation.y;
        q3 = msg->orientation.z;
        q0 = msg->orientation.w;

        double siny_cosp = 2 * (q0 * q3 + q1 * q2);
        double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        imu_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // Handle odometry data
    void handle_odom(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg)
    {
        rclcpp::Time curren_time = rclcpp::Clock().now();

        float now_odl = msg->data.at(0);
        float now_odr = msg->data.at(1);

        if (!is_initialized)
        {
            init_odl = now_odl;
            init_odr = now_odr;
            is_initialized = true;
        }

        now_odl -= init_odl;
        now_odr -= init_odr;

        dt = (curren_time - last_time_).seconds();
        last_time_ = curren_time;

        float dleft = now_odl - pre_odl;
        float dright = now_odr - pre_odr;

        pre_odl = now_odl;
        pre_odr = now_odr;

        float dxy_ave = (dright + dleft) / 2.0;
        float dth = (dright - dleft) / 0.175;
        vx = dxy_ave / dt;
        vw = dth / dt;

        if (dxy_ave != 0)
        {
            float dx = cos(dth / 2) * dxy_ave;
            float dy = sin(dth / 2) * dxy_ave;
            x_pos_ += (cos(yaw) * dx - sin(yaw) * dy);
            y_pos_ += (sin(yaw) * dx + cos(yaw) * dy);
        }

        if (dth != 0)
        {
            odom_yaw += dth;
        }

        //yaw = imu_yaw;
        yaw = odom_yaw;
    }

    // Publish odometry data
    void publish_odom()
    {
        rclcpp::Time curren_time = rclcpp::Clock().now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = curren_time;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_footprint_frame;

        // Robot's position in x, y, and z
        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.position.z = 0.0;

        // Robot's heading in quaternion
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(yaw / 2.0);
        odom.pose.pose.orientation.w = cos(yaw / 2.0);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vw;

        uint8_t i = 0;
        if (vx == 0 || vw == 0)
        { 
            for (i = 0; i < _countof(ODOM_POSE_COVARIANCE); i++)
            {
                odom.pose.covariance[i] = ODOM_POSE_COVARIANCE2[i];
            }
            for (i = 0; i < _countof(ODOM_TWIST_COVARIANCE); i++)
            {
                odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE2[i];
            }
        }
        else
        {
            for (i = 0; i < _countof(ODOM_POSE_COVARIANCE); i++)
            {
                odom.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
            }
            for (i = 0; i < _countof(ODOM_TWIST_COVARIANCE); i++)
            {
                odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
            }
        }

        odom_publisher_->publish(odom);

        if (pub_odom_tf_)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = curren_time;
            t.header.frame_id = odom_frame;
            t.child_frame_id = base_footprint_frame;
            t.transform.translation.x = x_pos_;
            t.transform.translation.y = y_pos_;
            t.transform.translation.z = 0.0;

            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = sin(yaw / 2.0);
            t.transform.rotation.w = cos(yaw / 2.0);
            tf_broadcaster_->sendTransform(t);
        }
    }
};

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
