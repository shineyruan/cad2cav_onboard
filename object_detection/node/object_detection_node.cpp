#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "object_detection/object_detector.hpp"

void imageReceiveCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  } catch (const std::exception& e) {
    ROS_ERROR(e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection_node");
  return 0;
}
