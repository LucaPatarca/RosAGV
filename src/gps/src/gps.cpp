#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "nmeaparse/nmea.h"

using namespace std::chrono_literals;

//GNGLL, GNGGA, GNRMC

class GpsNode : public rclcpp::Node
{
  public:
    GpsNode()
    : Node("gps_node"), gps_(parser_), device_("/dev/ttyACM0")
    {
      parser_.log = false;
      gps_.onUpdate += [this](){this->onGpsUpdate();};
      publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
      timer_ = this->create_wall_timer(
        10ms, std::bind(&GpsNode::readGpsLine, this));
      this->readGpsLine();
    }

  private:
    void readGpsLine()
    {
      std::string line;
      getline(device_, line);
      try {
        parser_.readLine(line);
      }
      catch (nmea::NMEAParseError& e){
        RCLCPP_ERROR(this->get_logger(), "NMEA parse error: '%s'", e.message.c_str());
      }
    }

    void onGpsUpdate()
    {
      sensor_msgs::msg::NavSatFix msg;
      if(gps_.fix.locked())
      {
        msg.altitude = gps_.fix.altitude;
        msg.latitude = gps_.fix.latitude;
        msg.longitude = gps_.fix.longitude;
        msg.status.status = msg.status.STATUS_FIX;
      } else 
      {
        RCLCPP_INFO(this->get_logger(), "GPS not locked.");
        msg.status.status = msg.status.STATUS_NO_FIX;
      }
      publisher_->publish(msg);
    }

    nmea::NMEAParser parser_;
    nmea::GPSService gps_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ifstream device_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsNode>());
  rclcpp::shutdown();
  return 0;
}