#ifndef __OBJECT_DETECTION_VIDEO_LOADER_HPP__
#define __OBJECT_DETECTION_VIDEO_LOADER_HPP__

/**
 * @file video_loader.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 *
 * @brief Video loader is responsible for all interactions with disk videos &
 * webcams video streams, including loading a video from disk, detecting
 * webcams, and saving videos to disk.
 *
 * @date 2021-06-13
 * @copyright Copyright (c) 2021
 */

#include <ros/console.h>

#include <chrono>
#include <object_detection/object_detector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>  // videoio.hpp included here
#include <opencv2/video/video.hpp>

namespace object_detection {

class VideoLoader {
public:
  /**
   * @brief Construct a new Video Loader object
   *
   * @param video_path:   absolute/relative path to video file
   *                          if use_webcam == false;
   *                      text-encoded number to webcam index
   *                          if use_webcam == true;
   * @param use_webcam:   whether to use webcam real-time video stream.
   *                      Calls different constructors of cv::VideoCapture
   */
  explicit VideoLoader(const std::string video_path, bool use_webcam);

  /**
   * @brief Gets the current frame of the video.
   *
   * @return const cv::Mat&
   */
  const cv::Mat& getFrame() const;

  /**
   * @brief Gets next frame from VideoCapture and sets the current frame
   *
   * @return true:    next frame has been retrieved
   * @return false:   no next frame available. Possibly reaches end of video
   */
  bool nextFrame();

  /**
   * @brief Saves the current frame to disk through cv::VideoWriter
   */
  void saveFrame();

  /**
   * @brief Saves a frame to video specified at input
   *
   * @param frame:  the frame to save
   *                (usually with detection bounding boxes)
   */
  void saveFrame(const cv::Mat& frame);

  /**
   * @brief Visualizes current frame using cv::imshow()
   */
  void visualize();

  /**
   * @brief Visualizes current frame with detection results
   *
   * @param bbox_list:    list of bounding boxes; detection result
   */
  void visualize(const std::vector<BoundingBox>& bbox_list);

  /**
   * @brief Visualizes a given frame
   *
   * @param frame
   */
  void visualize(const cv::Mat& frame);

private:
  cv::VideoCapture capture_;
  cv::VideoWriter writer_;
  int num_frames_;
  cv::Mat current_frame_;
  int frame_idx_;
  bool manual_termination_;

  // members for storing video properties
  int frame_width_;
  int frame_height_;
  double frame_fps_;
  std::string frame_fourcc_;

  /**
   * @brief Prints the properties of video from VideoCapture info using ROS
   * console
   *
   * @return int: the number of frames of the video (webcam stream return -1)
   */
  int displayVideoProperties();
};

}  // namespace object_detection

#endif /* __OBJECT_DETECTION_VIDEO_LOADER_HPP__ */
