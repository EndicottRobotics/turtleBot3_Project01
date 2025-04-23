#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
          tf_listener_(tf_buffer_)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&NavigateToTFTargetNode::controlLoop, this));

        // Initialize previous values
        prev_linear_vel_ = 0.0;
        prev_angular_vel_ = 0.0;
        dt_ = 0.1; // 100ms = 0.1s control loop interval
    }

private:
    void controlLoop()
    {
        geometry_msgs::msg::TransformStamped robot_tf, target_tf;

        try
        {
            // get the latest robot position and target data from the buffers
            robot_tf = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
            target_tf = tf_buffer_.lookupTransform("map", "target", tf2::TimePointZero);
        
            // Determine the age of the robot positon data
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time robot_tf_time = rclcpp::Time(robot_tf.header.stamp);
            double age = (now - robot_tf_time).seconds();
        
            // Warn if robot transform is stale and stop the robot(e.g., older than 1 second)
           /* if (age > 1.0) {
                RCLCPP_WARN(this->get_logger(), "Stale transform for robot: %.2f seconds old", age);
                //stop the robot something is wrong. 
                Robot_stop();
                return; 
            }*/
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"TF lookup failed: %s", ex.what());
                //stop the robot something is wrong. 
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

        // Raw control output (proportional controller)
        double desired_linear = (distance > 0.05) ? 0.3 * distance : 0.0;
        double desired_angular = (distance > 0.05) ? 0.8 * angle_error : 0.0;

        // Velocity limits
        double max_linear_vel;
        if(abs(angle_error) < .5){ max_linear_vel = 0.25;}
        else                { max_linear_vel = 0.0;}
        //const double max_linear_vel = 0.5;     // [m/s]
        const double max_angular_vel = 1.5;    // [rad/s]

        // Acceleration limits
        const double max_linear_acc = 0.4;     // [m/s^2]
        const double max_angular_acc = 1.0;    // [rad/s^2]

        double smoothed_linear = constrain_vel_and_acc(desired_linear, prev_linear_vel_, max_angular_vel, max_linear_acc );
        double smoothed_angular = constrain_vel_and_acc(desired_angular, prev_angular_vel_, max_angular_vel, max_angular_acc );

        // Publish smoothed command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = smoothed_linear;
        cmd_vel.angular.z = smoothed_angular;
        cmd_vel_pub_->publish(cmd_vel);

        // Update previous velocity for next cycle
        prev_linear_vel_ = smoothed_linear;
        prev_angular_vel_ = smoothed_angular;
    }
    double constrain_vel_and_acc(double desired_vel, double prev_vel, double max_V, double max_A){
    
        // Clamp desired velocities
        desired_vel = std::clamp(desired_vel, -max_V, max_V);
   
        // Apply acceleration limits
        double V_delta = desired_vel - prev_vel;
        double max_delta = max_A * dt_;
        V_delta = std::clamp(V_delta, -max_delta, max_delta);
    
        // return smoothed velocity 
        return(prev_vel + V_delta);
    }

    void Robot_stop(){
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub_->publish(cmd_vel);
        prev_linear_vel_ = 0;
        prev_angular_vel_ = 0;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Previous velocities for acceleration limiting
    double prev_linear_vel_;
    double prev_angular_vel_;
    double dt_; // Time between control loop iterations
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToTFTargetNode>());
    rclcpp::shutdown();
    return 0;
}