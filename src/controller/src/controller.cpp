#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
  public:
    Controller()
    : Node("controller")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/dolly/range_sensor", 10, std::bind(&Controller::range_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", 10);
    }

  private:

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
    {
      auto message = geometry_msgs::msg::Twist();
      if(msg->range < msg->max_range){
        message.linear.x = -1.0;
      }
      publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}