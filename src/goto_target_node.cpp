#include <cstdio>  // Standard Input and Output Library
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"  // Include turtlesim for the turtle's pose
#include "geometry_msgs/msg/pose.hpp"  // Include geometry_msgs for the target pose
#include "geometry_msgs/msg/twist.hpp"
#include "endicott_interfaces/msg/move_status.hpp" 
#include "../include/endicott_util.hpp"

// Include the custom MoveStatus.msg message
#include <cmath>
#include <memory>
#include <stdexcept>

class GotoNode : public rclcpp::Node {
public:
  GotoNode() : Node("goto_node") {

    // Subscription to command target Pose (geometry_msgs::msg::Pose)
      target_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
        "/target_pose", 10, std::bind(&GotoNode::target_pose_callback, this, std::placeholders::_1));
  
    // Subscription to the turtle's pose (turtlesim::msg::Pose)
      pose_subscription_  = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&GotoNode::pose_callback, this, std::placeholders::_1));

    // Publisher a comand vel msg to turtlesim
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Publisher a move status message
    goto_status_publisher_ = create_publisher<endicott_interfaces::msg::MoveStatus>("/goto_status", 10);

  }

private:

  void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Update target position based on the received pose message
    target_x     = msg->position.x;
    target_y     = msg->position.y;
    target_theta = msg->orientation.z;

    distance_to_target = cal_relative_polar_distance(target_x, target_y, current_x, current_y);
    angle_to_target    = cal_relative_polar_angle(target_x, target_y, current_x, current_y);
    //double angular_error = unwrap(angle_to_target - current_theta);

    // let the sequancer know we got the new target
    publish_goto_state("new_target", distance_to_target, angle_to_target );
    RCLCPP_INFO(this->get_logger(), " ----- new target: x=%.2f, y=%.2f -----", target_x, target_y);
  }

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_x     = msg->x;
    current_y     = msg->y;
    current_theta = msg->theta;
    move_to_target();
  }

  void move_to_target() {

    distance_to_target = cal_relative_polar_distance(target_x, target_y, current_x, current_y);
    angle_to_target    = cal_relative_polar_angle(target_x, target_y, current_x, current_y);
    double angular_error = unwrap(angle_to_target - current_theta);

    if(distance_to_target > 0.1){
      if(abs(angular_error) >0.1){
        rotate_turtle(angular_error);
        publish_goto_state("rotating to target", distance_to_target, angle_to_target );
        }
      else{
        move_forward_and_rotate(distance_to_target, angular_error);
        publish_goto_state("moving to target", distance_to_target, angle_to_target );
        }
      }
    else{
      double final_theta_err = unwrap(target_theta - current_theta);
      if (abs(final_theta_err) > 0.1){
        rotate_turtle(final_theta_err);
        publish_goto_state("orientation", distance_to_target, angle_to_target );
        }
      else{
        stop_turtle();
        publish_goto_state("at_target", distance_to_target, angle_to_target );
        }
    }
  }

  void rotate_turtle(double angular_error) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.angular.z = angular_error;
    velocity_publisher_->publish(twist_msg);
  }

  void move_forward_and_rotate(double distance, double angular_error) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = distance;
    twist_msg.angular.z = angular_error;
    velocity_publisher_->publish(twist_msg);
  }

  void stop_turtle() {
    geometry_msgs::msg::Twist twist_msg;
    velocity_publisher_->publish(twist_msg);
  }

  void publish_goto_state(const std::string& state, double distance_err, double angle_err ) {
    endicott_interfaces::msg::MoveStatus goto_status_msg;
    goto_status_msg.state          = state;
    goto_status_msg.distance_error = distance_err;
    goto_status_msg.theta_error    = angle_err;

    // Publish the message
    goto_status_publisher_->publish(goto_status_msg);
  }
  // Publisher and subscription variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<endicott_interfaces::msg::MoveStatus>::SharedPtr goto_status_publisher_;  // GotoState publisher
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscription_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;

   // Variables 
  double target_x     = 5.0;
  double target_y     = 5.0;
  double target_theta = 0.0;
  double current_x;
  double current_y;
  double current_theta;
  double distance_to_target;
  double angle_to_target;
  int    debug_info_cnt =0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto goto_node = std::make_shared<GotoNode>();

  rclcpp::spin(goto_node);

  rclcpp::shutdown();
  return 0;
}