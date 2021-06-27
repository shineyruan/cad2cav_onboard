#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "object_detection/video_loader.hpp"

static const cv::String KEYS =
    "{help h usage ?| | print this message                              }"
    "{video v       | | video specified for tracking. Default: webcam   }";

std::tuple<cv::String, bool> parseCmdInput(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, KEYS);
  parser.about("The video frame publishing node");
  if (parser.has("help")) {
    parser.printMessage();
    throw std::runtime_error("command line paring failed");
  }

  bool use_webcam         = !parser.has("video");
  unsigned int webcam_idx = 0;
  // video source indicator
  cv::String video_path = (use_webcam) ? std::to_string(webcam_idx)
                                       : parser.get<cv::String>("video");

  // parser error check
  if (!parser.check()) {
    parser.printErrors();
    throw std::runtime_error("command line paring failed");
  }

  return std::make_tuple(video_path, use_webcam);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publish_node");

  cv::String video_path;
  bool use_webcam;
  std::tie(video_path, use_webcam) = parseCmdInput(argc, argv);

  // init ROS image publishing utilies
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher img_publisher =
      it.advertise("/camera/image_raw", 1);

  object_detection::VideoLoader video_loader(video_path, use_webcam);

  while (ros::ok() && video_loader.nextFrame()) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    auto msg = cv_bridge::CvImage(header, "bgr8", video_loader.getFrame())
                   .toImageMsg();
    img_publisher.publish(msg);
    ros::spinOnce();
  }

  return 0;
}
