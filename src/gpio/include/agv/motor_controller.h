#include "geometry_msgs/msg/twist.hpp"

class MotorController
{
public:
  MotorController();
  ~MotorController();
  void setDirection(const geometry_msgs::msg::Twist::SharedPtr) const;
private:
  void goForward() const;
  void goBackward() const;
};