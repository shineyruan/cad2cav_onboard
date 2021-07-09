#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/utils/filesystem.hpp>

#include "object_detection/object_detector.hpp"

namespace object_detection {

class ImageReceiver {
public:
  ImageReceiver() = default;

private:
  ros::NodeHandle n_;
  cv::Mat current_frame_;
};

}  // namespace object_detection

void imageReceiveCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
  }

  cv::String package_dir = ros::package::getPath("object_detection");
  cv::String model_path =
                 cv::utils::fs::join(package_dir, "models/yolov4-tiny.weights"),
             config_path =
                 cv::utils::fs::join(package_dir, "models/yolov4-tiny.cfg");
  object_detection::ObjectDetector detector(
      model_path, object_detection::DNNType::DARKNET,
      object_detection::DatasetType::COCO, config_path);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection_node");
  return 0;
}
