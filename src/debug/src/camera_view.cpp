// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
// {

// }

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions options;
//   rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
//   cv::namedWindow("view");
//   cv::startWindowThread();
//   image_transport::ImageTransport it(node);
//   image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
//   rclcpp::spin(node);
//   cv::destroyWindow("view");

//   return 0;
// }

#include <memory>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class ImageListener : public rclcpp::Node
{
  public:
    ImageListener()
    : Node("image_listener"),
    logger_(rclcpp::get_logger("camera_view"))
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&ImageListener::callback, this, _1));
    }

  private:

    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {
        cv::InputArray data(msg->data);
        auto image = cv::imdecode(data, cv::COLOR_BGR2RGB);
        if(image.data == NULL){
          RCLCPP_ERROR(logger_, "Could not decode image.");
        }
        cv::imshow("view", image);
        cv::waitKey(10);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Logger logger_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageListener>());
  rclcpp::shutdown();
  return 0;
}