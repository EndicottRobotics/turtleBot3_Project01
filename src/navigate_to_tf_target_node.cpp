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
        double min_distance = 1000000; // make a very large number to start
        double angle_at_min_distance = 0;
        const double angle_threshold = M_PI / 4; // ±45 degrees

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::abs(angle) <= angle_threshold)
            {
                if (std::isfinite(msg->ranges[i]))
                {
                    if (min_distance > msg->ranges[i])
                    {
                        min_distance = msg->ranges[i];
                        angle_at_min_distance = angle;
                    }
                }
            }
        }
        obstacle_distance_ = min_distance;
        obstacle_angle_ = angle_at_min_distance;

        // Check for a valid (non-NaN) and close obstacle
        if (min_distance < 0.4) //  safety threshold
        {
            obstacle_detected_ = true;
        }
    }

    enum moveState
    {
        GO_TO_TARGET,
        TBD
    };
    moveState move_state =GO_TO_TARGET;

    void controlLoop()
    {
        if (!update_target_and_robot_pose())
        {
            Robot_stop();
            return;
        }
        switch (move_state)
        {

        case GO_TO_TARGET:
            rotate_then_move_to_target(target_x_, target_y_, current_x_, current_y_, current_theta_);
            break;

        case TBD:
            break;

        default:
            break;
        }
    }

    void rotate_then_move_to_target(double target_x, double target_y, double current_x, double current_y, double current_theta)
    {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::hypot(dx, dy);
        double angle_to_target = std::atan2(dy, dx);

        double angle_error = angle_to_target - current_theta;
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        // calculate the desired angular vel and constarin it
        double desired_angular = (distance > in_pos_err_) ? 0.8 * angle_error : 0.0;

        // calculate the desired linear speed base on angle to target, distance to target, constains and obstacle distance
        double desired_linear;
        if (std::abs(angle_error) > 0.2)
        {
            desired_linear = 0; // get pointed at target before starting to move forward
        }
        else
        {
            // check to see if we are coming up on a obstacle
            if (obstacle_detected_)
            {
                RCLCPP_WARN(this->get_logger(), "obstacle: obj_ang %.2f, obj_dis %.2f, trg_dis %.2f", obstacle_angle_, obstacle_distance_, distance);
                double temp = obstacle_distance_ - obstacle_safty_distance_;
                if (temp < 0)
                {
                    temp = 0;
                }
                if (distance > temp)
                {
                    distance = temp;
                }
            }
            desired_linear = (distance > in_pos_err_) ? 0.3 * distance : 0.0;
        }
        constratin_vel_acc_and_publish_vel_cmd(desired_angular, desired_linear);
    }

    void constratin_vel_acc_and_publish_vel_cmd(double desired_angular, double desired_linear)
    {
        // constrain the rate of accleration and max vel
        double smoothed_angular = constrain_vel_and_acc(desired_angular, prev_angular_vel_, max_angular_vel_, max_angular_acc_);
        double smoothed_linear = constrain_vel_and_acc(desired_linear, prev_linear_vel_, max_linear_vel_, max_linear_acc_);

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

    bool update_target_and_robot_pose()
    {

        geometry_msgs::msg::TransformStamped robot_tf, target_tf;

        // get latest target info and see if it is a new target
        try
        {
            target_tf = tf_buffer_.lookupTransform("map", "target", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            return false;
        }
        bool new_target_ = false;
        if (target_x_ != target_tf.transform.rotation.x)
        {
            target_x_ = target_tf.transform.rotation.x;
            new_target_ = true;
        }
        if (target_y_ != target_tf.transform.rotation.y)
        {
            target_y_ = target_tf.transform.rotation.y;
            new_target_ = true;
        }

        // get latest robot position
        try
        {
            robot_tf = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No robot TF: %s", ex.what());
            return false;
        }
        tf2::Quaternion q(robot_tf.transform.rotation.x, robot_tf.transform.rotation.y,
                          robot_tf.transform.rotation.z, robot_tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_x_ = robot_tf.transform.rotation.x;
        current_y_ = robot_tf.transform.rotation.y;
        current_theta_ = yaw;
        return true;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double prev_linear_vel_;
    double prev_angular_vel_;
    double target_x_;
    double target_y_;
    bool new_target_;
    double current_x_;
    double current_y_;
    double current_theta_;
    double dt_;
    bool obstacle_detected_;
    double obstacle_distance_;
    double obstacle_angle_;
    double max_angular_vel_ = 1.5;
    double max_angular_acc_ = 1.0;
    double max_linear_vel_ = 0.25;
    double max_linear_acc_ = 0.4;
    double in_pos_err_ = 0.05;
    double obstacle_safty_distance_ = 0.3;
    double min_stop_distance_at_max_v = obstacle_safty_distance_ + (max_linear_vel_ * max_linear_vel_ / (2 * max_linear_acc_));
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToTFTargetNode>());
    rclcpp::shutdown();
    return 0;
}
