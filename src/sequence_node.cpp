#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"  
#include "turtlesim/msg/color.hpp"
#include "endicott_interfaces/msg/move_status.hpp"
#include "../include/endicott_util.hpp"
#include <cmath>
#include <memory>
#include <random>

#define dT  20 // set to 20 milliseconds 


class SequenceNode : public rclcpp::Node {
public:

  SequenceNode() : Node("sequence_node") {
    // Create a publisher for the /target_pose topic
    target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);

    // Create a subscription to the /goto_status topic
    goto_status_subscription_ = create_subscription<endicott_interfaces::msg::MoveStatus>(
      "/goto_status", 10, std::bind(&SequenceNode::goto_status_callback, this, std::placeholders::_1));

    // Subscription to the turtle's pose (turtlesim::msg::Pose)
    pose_subscription_  = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&SequenceNode::pose_callback, this, std::placeholders::_1));
  
    // Subscription to the turtle's color sensor (turtlesim::msg::Color)
    color_subscription_ = create_subscription<turtlesim::msg::Color>(
      "/turtle1/color_sensor", 10, std::bind(&SequenceNode::color_callback, this, std::placeholders::_1));

    // Create a timer that will call the timer_callback() every 1 second
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(dT), std::bind(&SequenceNode::timer_callback, this));

  }

private:
  // Timer callback that will periodically call the clean_rectangle and publish target pose
  void timer_callback() {

    switch(task_cnt){
      case(0):
        task_cnt++;
        RCLCPP_INFO(this->get_logger(), "Time to clean");
        init_random_clean_rectangle(); 
        break;

      case(1):
        //clean room xxx
        if(random_clean_rectangle(1, 1, 5, 5, 20)){  
          task_cnt++;  
          RCLCPP_INFO(this->get_logger(), "Done cleaning room xxx");
          init_random_clean_rectangle(); 
        }
        break;

      case(2):
        //clean room yyy
        if(random_clean_rectangle(5, 5, 10, 10, 20)){  
          task_cnt++;  
          RCLCPP_INFO(this->get_logger(), "Done cleaning room yyy");
        }
        break;

      case(3):
        //send command to move back home home
        target_x = 5;
        target_y = 5;
        target_theta = 0;
        publish_target_pose(target_x, target_y, target_theta);
        RCLCPP_INFO(this->get_logger(), "Going home to charge");
        task_cnt++;
        break;

      case(4):
        //wait for robot to get home
        if(check_if_at_target(0.1, 10000)){ 
          task_cnt++;
          RCLCPP_INFO(this->get_logger(), "Back to home base and Done with cleaning");
        }
        break;

      default:
        break;
    }
  }

  enum CleanSeqState{SEND_NEW_MOVE, WAIT_TO_REACH_TARGET,AT_TARGET,DONE};

  // Functions to simulate cleaning by publishing random target positions
  void init_random_clean_rectangle() {
    clean_seq_state = SEND_NEW_MOVE;
    move_cnt = 0;
  }

  bool random_clean_rectangle(double x1, double y1, double x2, double y2, int max_moves) {
    
    switch(clean_seq_state) {

      case SEND_NEW_MOVE:
        target_x = x1 + (std::rand() % (int)(x2-x1));
        target_y = y1 + (std::rand() % (int)(y2-y1));
        target_theta = 0; // not use but might add in future
        publish_target_pose(target_x, target_y , target_theta);
        clean_seq_state = WAIT_TO_REACH_TARGET;
        return(false);

      case WAIT_TO_REACH_TARGET:
        if(check_if_at_target(0.1, 10000)){
          clean_seq_state = AT_TARGET;
        }
        return(false);

      case AT_TARGET:
        if(move_cnt == max_moves){
          clean_seq_state = DONE;
        }
        else{
          move_cnt++;
          clean_seq_state = SEND_NEW_MOVE;
        }
        return(false);

      case DONE:
        return(true);

    }
    RCLCPP_INFO(this->get_logger(), "ERROR: SHOUND NEVER BE AT THIS LINE OF CODE");
    return(false);
  }


  // check to see if robot is at target 
  // return true if it is eles return false
  bool check_if_at_target(double distance_tolerance, double angle_tolerance) {
    double distance_to_target = cal_relative_polar_distance(target_x, target_y, current_x, current_y);
    double angle_to_target    = cal_relative_polar_angle(target_x, target_y, current_x, current_y);
    double angular_error = unwrap(angle_to_target - current_theta);
    return(((distance_to_target < distance_tolerance) && (abs(angular_error) < angle_tolerance)));
  }

  // Function to publish target pose to the /target_pose topic
  void publish_target_pose(double x, double y, double theta) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.orientation.z = theta;

    // Log and publish the message
    //RCLCPP_INFO(this->get_logger(), "Publishing target pose: x=%.2f, y=%.2f", target_pose.position.x, target_pose.position.y);
    target_pose_publisher_->publish(target_pose);
  }

  // pose_callback 
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_theta = msg->theta;
  }

  // color_callback 
  void color_callback(const turtlesim::msg::Color::SharedPtr msg) {
    color_r = msg->r, 
    color_g = msg->g, 
    color_b = msg->b;
  }

  // Callback function for the /goto_status topic
  void goto_status_callback(const endicott_interfaces::msg::MoveStatus::SharedPtr msg) {
    goto_distance_err = msg->distance_error;
    goto_angular_err  = msg->theta_error;
  }

  // Publisher and subscription variables
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr                target_pose_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr                 pose_subscription_;
  rclcpp::Subscription<turtlesim::msg::Color>::SharedPtr                color_subscription_;
  rclcpp::Subscription<endicott_interfaces::msg::MoveStatus>::SharedPtr goto_status_subscription_;

  // Timer for periodic tasks
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables  
  double          current_x;
  double          current_y;
  double          current_theta;
  double          target_x;
  double          target_y;
  double          target_theta;
  int             task_cnt;
  int             move_cnt;
  uint            color_r;
  uint            color_g;
  uint            color_b; 
  CleanSeqState   clean_seq_state;
  double          goto_distance_err;
  double          goto_angular_err;
};

int main(int argc, char *argv[]) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);
  auto sequence_node = std::make_shared<SequenceNode>();

  // Create and spin the TargetSequenceNode
  rclcpp::spin(sequence_node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}