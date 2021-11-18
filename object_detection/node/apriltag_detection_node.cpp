#include <ros/ros.h>

#include "object_detection/landmark_processor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detection_node");

  object_detection::LandmarkProcessor landmark_processor{"Tag36h11", 0.168, true};
  ros::spin();

  return 0;
}
