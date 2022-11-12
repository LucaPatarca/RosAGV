#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <agv/motor_controller.h>

class RangeNode : public rclcpp::Node{
public:
  RangeNode()
  : Node("movement"){
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&RangeNode::vel_callback, this, std::placeholders::_1));
  }

private:
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
    controller_.setDirection(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  MotorController controller_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeNode>());
  rclcpp::shutdown();
  return 0;
}