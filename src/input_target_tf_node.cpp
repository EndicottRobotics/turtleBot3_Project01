#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <memory>
#include <cmath>

class InputTargetTFNode : public rclcpp::Node
{
public:
    InputTargetTFNode()
    : Node("input_target_tf_node")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        prompt_and_publish();
    }

private:
    void prompt_and_publish()
    {
        while (rclcpp::ok()) {
            double x, y, theta_deg;

            std::cout << "Enter target coordinates (x y theta): ";
            std::cin >> x >> y >> theta_deg;

            double theta_rad = theta_deg * M_PI / 180.0;

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id = "target";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, theta_rad);
            q.normalize();

            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf_msg);

            RCLCPP_INFO(this->get_logger(),
                        "Published target TF: (x=%.2f, y=%.2f, theta=%.1f°)",
                        x, y, theta_deg);
        }
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputTargetTFNode>());
    rclcpp::shutdown();
    return 0;
}