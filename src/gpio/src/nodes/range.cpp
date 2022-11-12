#include <memory>
#include <agv/range_sensor.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class RangeSensorNode : public rclcpp::Node
{
public:
        RangeSensorNode()
            : Node("range")
        {
                using namespace std::chrono_literals;
                publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range", 10);
                timer_ = this->create_wall_timer(
                    350ms, std::bind(&RangeSensorNode::timer_callback, this));
        }

private:
        void timer_callback()
        {
                auto message = sensor_msgs::msg::Range();
                message.range = sensor_.getDistance();
                publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
        RangeSensor sensor_;
};

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<RangeSensorNode>());
        rclcpp::shutdown();
        return 0;
}
