#include <ros/ros.h>

#include "object_detection/landmark_processor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection_node");

  object_detection::LandmarkProcessor landmark_processor{};
  ros::spin();

  return 0;
}
