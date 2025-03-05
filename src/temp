#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"  // For Pose message type

class TargetPublisherNode : public rclcpp::Node {
public:
  TargetPublisherNode() : Node("target_publisher_node") {
    // Create a publisher for the /target_pose topic
    target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);

    // Set a timer to periodically publish a target pose
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&TargetPublisherNode::publish_target_pose, this));
  }

private:
  void publish_target_pose() {
    // Create a new Pose message
    geometry_msgs::msg::Pose target_pose;

    // Set the target position (you can change these values to test different targets)
    target_pose.position.x = 5.0;  // Target X position
    target_pose.position.y = 5.0;  // Target Y position
    target_pose.orientation.w = 1.0;  // Set a default orientation (no rotation)

    // Log the message being published
    RCLCPP_INFO(this->get_logger(), "Publishing target pose: x=%.2f, y=%.2f", target_pose.position.x, target_pose.position.y);

    // Publish the target pose
    target_pose_publisher_->publish(target_pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
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