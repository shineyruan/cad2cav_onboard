#include <object_detection/video_loader.hpp>
#include <opencv2/core/utils/filesystem.hpp>

using namespace std::chrono;

namespace object_detection {

VideoLoader::VideoLoader(const std::string video_path, bool use_webcam)
    : frame_idx_(0), manual_termination_(false) {
  if (use_webcam) {
    ROS_INFO_STREAM("Using webcam as video source...");
  } else {
    ROS_INFO_STREAM("Retrieving video according to path: " << video_path);
  }

  bool stream_successful = (use_webcam) ? capture_.open(std::stoi(video_path))
                                        : capture_.open(video_path);
  if (use_webcam) {
    if (!stream_successful)
      ROS_FATAL("Webcam %d not connected. Please verify!\n",
                std::stoi(video_path));
  } else if (!stream_successful) {
    ROS_FATAL_STREAM("Filename '" << video_path << "' not found. "
                                  << "Please verify!\n");
  }

  num_frames_ = capture_.isOpened() ? displayVideoProperties() : -1;

  // for video saving
  auto now        = system_clock::now();
  int file_suffix = static_cast<int>(system_clock::to_time_t(now));
  int codec       = cv::VideoWriter::fourcc(frame_fourcc_[0], frame_fourcc_[1],
                                      frame_fourcc_[2], frame_fourcc_[3]);
  std::string write_dir = "video_out";
  if (!cv::utils::fs::exists(write_dir))
    cv::utils::fs::createDirectory(write_dir);
  writer_ = cv::VideoWriter(
      cv::format("%s/video-%d.mp4", write_dir.c_str(), file_suffix), codec,
      frame_fps_, cv::Size(frame_width_, frame_height_));
}

const cv::Mat& VideoLoader::getFrame() const { return current_frame_; }

bool VideoLoader::nextFrame() {
  bool is_videoEnd = (num_frames_ >= 0) ? (frame_idx_ >= num_frames_) : false;
  if (capture_.isOpened() && !manual_termination_ && !is_videoEnd) {
    capture_ >> current_frame_;
    ++frame_idx_;
    return true;
  }

  return false;
}

void VideoLoader::saveFrame() { writer_.write(current_frame_); }

void VideoLoader::saveFrame(const cv::Mat& frame) { writer_.write(frame); }

int VideoLoader::displayVideoProperties() {
  int num_frames = capture_.get(cv::CAP_PROP_FRAME_COUNT);
  ROS_INFO_STREAM("\t Using OpenCV version: " << cv::getVersionString());
  ROS_INFO_STREAM("\t Number of Frames: " << num_frames);

  // detect video properties
  frame_width_  = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
  frame_height_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
  frame_fps_    = capture_.get(cv::CAP_PROP_FPS);

  if (num_frames > 0) {
    int fourcc    = capture_.get(cv::CAP_PROP_FOURCC);
    frame_fourcc_ = cv::format("%c%c%c%c", fourcc & 255, (fourcc >> 8) & 255,
                               (fourcc >> 16) & 255, (fourcc >> 24) & 255);
  } else {
    frame_fourcc_ = "mp4v";
  }

  ROS_INFO_STREAM("\t Width: " << frame_width_);
  ROS_INFO_STREAM("\t Height: " << frame_height_);
  ROS_INFO_STREAM("\t FourCC: " << frame_fourcc_);
  ROS_INFO_STREAM("\t Frame rate: " << frame_fps_);

  return num_frames;
}

void VideoLoader::visualize() {
  if (current_frame_.empty()) {
    ROS_ERROR("Empty frame %d for display!", frame_idx_);
    return;
  }

  cv::imshow("Video Source", current_frame_);
  char ch             = cv::waitKey(30);
  manual_termination_ = (ch == 'q' || ch == 'Q') ? true : false;
}

void VideoLoader::visualize(const std::vector<BoundingBox>& bbox_list) {
  for (const auto& bbox : bbox_list) {
    cv::putText(current_frame_, bbox.class_name, cv::Point(bbox.left, bbox.top),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::rectangle(current_frame_, bbox.getBBox(), cv::Scalar(0, 0, 255));
  }

  visualize();
}

}  // namespace object_detection
