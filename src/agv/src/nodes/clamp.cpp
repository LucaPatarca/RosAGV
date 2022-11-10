#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <agv/clamp_controller.h>

class ClampNode : public rclcpp::Node{
public:
  ClampNode()
  : Node("clamp"){
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/clamp", 10, std::bind(&ClampNode::clamp_callback, this, std::placeholders::_1));
  }

private:
  void clamp_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) const {
    controller_.setPosition(msg->z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
  ClampController controller_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClampNode>());
  rclcpp::shutdown();
  return 0;
}