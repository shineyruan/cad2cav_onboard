#include <ros/ros.h>
#include <signal.h>

#include <object_detection/fps.hpp>
#include <object_detection/video_loader.hpp>

static const cv::String KEYS =
    "{help h usage ?| | print this message                              }"
    "{video v       | | video specified for tracking. Default: webcam   }"
    "{save s        | | whether to save video to disk                   }"
    "{visualize     | | whether to visualize each frame in window       }";

bool termination = false;
void onShutDown(int sig) {
  termination = true;
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "video_recording_node");

  // OpenCV command line parser
  cv::CommandLineParser parser(argc, argv, KEYS);
  parser.about("The object detection node");
  if (parser.has("help")) {
    parser.printMessage();
    return EXIT_SUCCESS;
  }

  // determine using webcam or video
  bool use_webcam         = !parser.has("video");
  bool save_video         = parser.has("save");
  bool viz_enable         = parser.has("visualize");
  unsigned int webcam_idx = 0;

  // video source indicator
  cv::String video_path = (use_webcam) ? std::to_string(webcam_idx)
                                       : parser.get<cv::String>("video");

  // parser error check
  if (!parser.check()) {
    parser.printErrors();
    return EXIT_FAILURE;
  }

  object_detection::VideoLoader video_loader(video_path, use_webcam);

  object_detection::FPS fps_counter;
  fps_counter.start();

  ROS_WARN_STREAM(
      "Use Ctrl-C to terminate when visualization is not enabled; use q or Q "
      "when visualization is enabled");

  // register SIGINT callback
  signal(SIGINT, onShutDown);

  while (video_loader.nextFrame() && !termination) {
    if (viz_enable) video_loader.visualize();
    if (save_video) video_loader.saveFrame();
    fps_counter.update();
  }

  fps_counter.stop();
  fps_counter.printFPSMessage();

  return EXIT_SUCCESS;
}
