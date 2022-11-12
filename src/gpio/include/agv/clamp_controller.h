#include "geometry_msgs/msg/twist.hpp"

class ClampController
{
public:
  ClampController();
  ~ClampController();
  void setPosition(const double value) const;
private:
};