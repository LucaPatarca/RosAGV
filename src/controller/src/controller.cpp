#include <memory>
#include <cstdlib>
#include <ctime>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "std_msgs/msg/bool.hpp"

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
      objectSub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/object/detection", 10, std::bind(&Controller::object_callback, this, _1));
      velPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      clampPub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/clamp", 10);
      objectPub_ = this->create_publisher<std_msgs::msg::Bool>("/object/request", 10);
      ledPub_ = this->create_publisher<std_msgs::msg::Bool>("/led", 10);
      lastDirChange_ = std::chrono::steady_clock::now();
      lastDir_ = 1.0;
      waitForObject_ = false;
    }

  private:
    void object_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
      lastObjectDetection_ = std::chrono::steady_clock::now();
      bool foundBall = false;
      for (vision_msgs::msg::Detection2D detection : msg->detections){
        RCLCPP_INFO(this->get_logger(), "detection: '%s'", detection.results[0].id.c_str());
        if(detection.results[0].id == "sports ball"){
          foundBall = true;
        }
      }
      if(foundBall){
        geometry_msgs::msg::Vector3 clampMessage;
        clampPub_->publish(clampMessage);
      }
      std_msgs::msg::Bool ledMsg;
      ledMsg.data = false;
      ledPub_->publish(ledMsg);
      rclcpp::sleep_for(std::chrono::nanoseconds(2000000000));
      waitForObject_ = false;
    }

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      if(waitForObject_) return;
      geometry_msgs::msg::Twist velMessage;

      auto lastChangeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastDirChange_).count();
      auto lastObjectSeconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - lastObjectDetection_).count();

      if(msg->range > 40){
        velMessage.linear.x = 0.35;
      } else if (msg->range < 23 && lastObjectSeconds > 5){
        velMessage.linear.x = 0;
        geometry_msgs::msg::Vector3 clampMessage;
        clampMessage.z = 1.0;
        clampPub_->publish(clampMessage);

        std_msgs::msg::Bool objReq;
        objReq.data = true;
        objectPub_->publish(objReq);
        RCLCPP_INFO(this->get_logger(), "sent request");
        
        std_msgs::msg::Bool ledMsg;
        ledMsg.data = true;
        ledPub_->publish(ledMsg);
        waitForObject_ = true;
      } else {
        velMessage.linear.x = -0.35;
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
      velPub_->publish(velMessage);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangeSub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr objectSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr clampPub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr objectPub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ledPub_;
    std::chrono::steady_clock::time_point lastDirChange_;
    double lastDir_;
    bool waitForObject_;
    std::chrono::steady_clock::time_point lastObjectDetection_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
