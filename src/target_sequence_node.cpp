#include <cstdio> // Standard Input and Output Library
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"  // Include turtlesim for the turtle's pose
#include "geometry_msgs/msg/pose.hpp"  // Include geometry_msgs for the target pose
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>
#include <random>



class TargetPublisherNode : public rclcpp::Node {
public:
  TargetPublisherNode() : Node("target_publisher_node") {
    // Create a publisher for the /target_pose topic
    target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
    while(true)  {
      //move to the starting position
      TargetPublisherNode::publish_target_pose( x1, y1, 0);
      rclcpp::sleep_for(std::chrono::seconds(10));

      //clean the rectangle
      clean_rectangle( x1, y1, x2, y2);
    }
  }

private:

void clean_rectangle( double x1, double y1, double x2, double y2) {

  for (int n=0; n<10; n++){
    int range = x2-x1;
    int random_x = x1 + (std::rand() % range);
    int random_y = y1 + (std::rand() % range);
    TargetPublisherNode::publish_target_pose( random_x, random_y, 0);
    rclcpp::sleep_for(std::chrono::seconds(10));
  }
}

  void publish_target_pose( double x, double y, double theta) {
    // Create a new Pose message
    geometry_msgs::msg::Pose target_pose;

    // goto the lower right conner 
    target_pose.position.x = x;  
    target_pose.position.y = y; 
    target_pose.orientation.z = theta; 
    
    //display the message to the command window for debug
    RCLCPP_INFO(this->get_logger(), "Publishing target pose: x=%.2f, y=%.2f", target_pose.position.x, target_pose.position.y); 

    // Publish the target pose
    target_pose_publisher_->publish(target_pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x1 = 2;
  double y1 = 2;
  double x2 = 9;
  double y2 = 9;
};

int main(int argc, char *argv[]) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create and spin the target publisher node
  rclcpp::spin(std::make_shared<TargetPublisherNode>());

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}