#include <memory>
#include <cstdlib>
#include <ctime>
#include <chrono>

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
      srand(time(NULL));
      rangeSub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/range", 10, std::bind(&Controller::range_callback, this, _1));
      velPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      clampPub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/clamp", 10);
      lastDirChange_ = std::chrono::steady_clock::now();
      lastDir_ = 1.0;
    }

  private:

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      auto velMessage = geometry_msgs::msg::Twist();
      auto clampMessage = geometry_msgs::msg::Vector3();

      auto lastChangeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastDirChange_).count();

      clampMessage.z = 1.0;
      if(msg->range > 35){
        velMessage.linear.x = 0.25;
      } else if (msg->range < 10){
        clampMessage.z = 0.0;
        velMessage.linear.x = -0.25;
      } else {
        velMessage.linear.x = -0.25;
        if(lastChangeMillis > 5000){
          int dir = rand();
          if(dir > RAND_MAX / 2)
            lastDir_ = 1.0;
          else
            lastDir_ = -1.0;
          lastDirChange_ = std::chrono::steady_clock::now();
        }
        velMessage.angular.z = lastDir_;
      }
      // if(msg->range < 10){
	    //   clampMessage.z = 0.0;
      // }
      velPub_->publish(velMessage);
      clampPub_->publish(clampMessage);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangeSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr clampPub_;
    std::chrono::steady_clock::time_point lastDirChange_;
    double lastDir_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
