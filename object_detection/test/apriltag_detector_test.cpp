
#include "object_detection/apriltag_detector.hpp"

#include <ros/package.h>
#include <ros/ros.h>

#include "object_detection/video_loader.hpp"

static const cv::String KEYS =
    "{help h usage ?| | print this message                              }"
    "{video v       | | video specified for tracking. Default: webcam   }";

int main(int argc, char **argv) {
  // OpenCV command line parser
  cv::CommandLineParser parser(argc, argv, KEYS);
  parser.about("The object detection node");
  if (parser.has("help")) {
    parser.printMessage();
    return EXIT_SUCCESS;
  }

  // determine using webcam or video
  bool use_webcam         = !parser.has("video");
  unsigned int webcam_idx = 0;

  // video source indicator
  cv::String video_path = (use_webcam) ? std::to_string(webcam_idx)
                                       : parser.get<cv::String>("video");

  // parser error check
  if (!parser.check()) {
    parser.printErrors();
    return EXIT_FAILURE;
  }

  // init camera params
  const std::string camera_params_path = "camera/params.xml";
  cv::Mat cam_intrinsic;
  cv::FileStorage file(camera_params_path, cv::FileStorage::READ);
  if (!file.isOpened()) {
    ROS_FATAL("Cannot open camera calibration params at %s",
              camera_params_path.c_str());
    throw ros::Exception("camera param read error");
  }
  file["intrinsic"] >> cam_intrinsic;
  file.release();

  // init video loader & apriltag detector
  const std::string tag_family = "TagStandard41h12";
  const double tag_size        = 0.01495;
  object_detection::VideoLoader video_loader(video_path, use_webcam);
  object_detection::AprilTagDetector tag_detector(tag_family, tag_size,
                                                  cam_intrinsic);

  while (video_loader.nextFrame()) {
    const auto detections = tag_detector.detect(video_loader.getFrame());
    video_loader.visualize(
        tag_detector.visualizeTags(detections, video_loader.getFrame()));
    if (!detections.empty()) {
      // right X, down Y, forward Z
      auto unit_rotation = cv::Mat::eye(3, 3, CV_32F);
      cv::Mat unit_rotation_vec;
      cv::Rodrigues(unit_rotation, unit_rotation_vec);
      ROS_INFO_STREAM("Tag pose: " << detections.front().pos
                                   << "; Tag rotation: "
                                   << detections.front().rot);
    }
  }

  return EXIT_SUCCESS;
}
