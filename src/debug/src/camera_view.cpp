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
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <memory>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

cv::Rect2d bboxToRect(vision_msgs::msg::BoundingBox2D bbox)
{
  cv::Point2d point1(bbox.center.x,bbox.center.y);
  cv::Point2d point2(bbox.center.x + bbox.size_x,bbox.center.y + bbox.size_y);
  cv::Rect2d rect(point1,point2);
  return rect;
}

class ImageListener : public rclcpp::Node
{
  public:
    ImageListener()
    : Node("image_listener"),
    logger_(rclcpp::get_logger("camera_view"))
    {
      imageSubscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10, std::bind(&ImageListener::imageCallback, this, _1));
      detectionSubscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/agv/detections", 10, std::bind(&ImageListener::detectionCallback, this, _1));
      cv::namedWindow("Camera View", cv::WindowFlags::WINDOW_KEEPRATIO);
    }

  private:

    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {
        cv::InputArray data(msg->data);
        auto image = cv::imdecode(data, cv::COLOR_BGR2RGB);
        if(image.data == NULL){
          RCLCPP_ERROR(logger_, "Could not decode image.");
        }
        for(vision_msgs::msg::Detection2D detection : lastDetection_->detections)
        {
          cv::rectangle(image, bboxToRect(detection.bbox), 0xFF0000, 2);
          if(detection.results.size() > 0)
          {
            cv::putText(image, detection.results[0].id, cv::Point(detection.bbox.center.x-10, detection.bbox.center.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, 0xFF0000, 2);
          }
        }
        cv::imshow("Camera View", image);
        cv::waitKey(10);
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
      lastDetection_ = msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSubscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detectionSubscription_;
    vision_msgs::msg::Detection2DArray::SharedPtr lastDetection_;
    rclcpp::Logger logger_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageListener>());
  rclcpp::shutdown();
  return 0;
}