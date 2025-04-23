#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class NavigateToTFTargetNode : public rclcpp::Node
{
public:
    NavigateToTFTargetNode()
        : Node("navigate_to_tf_target_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          obstacle_detected_(false)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NavigateToTFTargetNode::scanCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&NavigateToTFTargetNode::controlLoop, this));

        prev_linear_vel_ = 0.0;
        prev_angular_vel_ = 0.0;
        dt_ = 0.1;
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    obstacle_detected_ = false;
    const double angle_threshold = M_PI / 4;  // ±45 degrees

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        double angle = msg->angle_min + i * msg->angle_increment;
        if (std::abs(angle) <= angle_threshold)
        {
            double distance = msg->ranges[i];
            // Check for a valid (non-NaN) and close obstacle
            if (std::isfinite(distance) && distance < 0.2)  // 0.2 meters is a safety threshold
            {
                obstacle_detected_ = true;
                RCLCPP_WARN(this->get_logger(), "Obstacle detected at angle %.2f, distance %.2f", angle, distance);
                return;
            }
        }
    }
}

    void controlLoop()
    {

        geometry_msgs::msg::TransformStamped robot_tf, target_tf;

        try
        {
            robot_tf = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
            target_tf = tf_buffer_.lookupTransform("map", "target", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF lookup failed: %s", ex.what());
            Robot_stop();
            return;
        }

        double dx = target_tf.transform.translation.x - robot_tf.transform.translation.x;
        double dy = target_tf.transform.translation.y - robot_tf.transform.translation.y;
        double distance = std::hypot(dx, dy);
        double angle_to_target = std::atan2(dy, dx);

        tf2::Quaternion q(
            robot_tf.transform.rotation.x,
            robot_tf.transform.rotation.y,
            robot_tf.transform.rotation.z,
            robot_tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double angle_error = angle_to_target - yaw;
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        
        // calculate the desired speed base on distance to target
        double desired_linear = (distance > 0.05) ? 0.3 * distance : 0.0;
        double desired_angular = (distance > 0.05) ? 0.8 * angle_error : 0.0;

       // set vel and acc contraints
        double max_linear_vel  = 0.25;
        // stop forward motion if a obstacle is detected in front of the robot 
        // or the angle_error is to big
        if ((obstacle_detected_) || (std::abs(angle_error) < 0.2)) { max_linear_vel = 0;}
        const double max_angular_vel = 1.5;
        const double max_linear_acc = 0.4;
        const double max_angular_acc = 1.0;

        // constrain the vel and acc of the robot
        double smoothed_linear = constrain_vel_and_acc(desired_linear, prev_linear_vel_, max_linear_vel, max_linear_acc);
        double smoothed_angular = constrain_vel_and_acc(desired_angular, prev_angular_vel_, max_angular_vel, max_angular_acc);

        // send the new vel command to the robot
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = smoothed_linear;
        cmd_vel.angular.z = smoothed_angular;
        cmd_vel_pub_->publish(cmd_vel);

        prev_linear_vel_ = smoothed_linear;
        prev_angular_vel_ = smoothed_angular;
    }

    double constrain_vel_and_acc(double desired_vel, double prev_vel, double max_V, double max_A)
    {
        desired_vel = std::clamp(desired_vel, -max_V, max_V);
        double V_delta = desired_vel - prev_vel;
        double max_delta = max_A * dt_;
        V_delta = std::clamp(V_delta, -max_delta, max_delta);
        return prev_vel + V_delta;
    }

    void Robot_stop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub_->publish(cmd_vel);
        prev_linear_vel_ = 0;
        prev_angular_vel_ = 0;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double prev_linear_vel_;
    double prev_angular_vel_;
    double dt_;
    bool obstacle_detected_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToTFTargetNode>());
    rclcpp::shutdown();
    return 0;
}
