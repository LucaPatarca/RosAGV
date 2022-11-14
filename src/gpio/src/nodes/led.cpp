#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wiringPi.h"

#define LED_PIN 0

class LedNode : public rclcpp::Node{
public:
  LedNode()
  : Node("led"){
    if (wiringPiSetup() == -1)
    {
      exit(EXIT_FAILURE);
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/led", 10, std::bind(&LedNode::led_callback, this, std::placeholders::_1));
  }

  ~LedNode(){
    digitalWrite(LED_PIN, LOW);
  }

private:
  void led_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
    digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedNode>());
  rclcpp::shutdown();
  return 0;
}