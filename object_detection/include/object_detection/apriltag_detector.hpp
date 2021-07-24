#ifndef __OBJECT_DETECTION_APRILTAG_DETECTOR_HPP__
#define __OBJECT_DETECTION_APRILTAG_DETECTOR_HPP__

/**
 * @file apriltag_detector.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 *
 * @brief Apriltag detector is a C++ wrapper for Apriltag library. It provides a
 * single-type tag detector & pose estimator with OpenCV interfaces.
 *
 * For more information on Apriltag, see
 * https://april.eecs.umich.edu/software/apriltag
 *
 * @date 2021-07-12
 */

#include "object_detection/apriltag.hpp"

namespace object_detection {

class AprilTagDetector {
public:
  AprilTagDetector(apriltag::TagType tag_type, double tag_size, double fx,
                   double fy, double cx, double cy);
  AprilTagDetector(apriltag::TagType tag_type, double tag_size,
                   const cv::Mat& intrinsic_mtx);
  AprilTagDetector(const std::string tag_type_str, double tag_size,
                   const cv::Mat& intrinsic_mtx);
  ~AprilTagDetector();

  std::vector<apriltag::TagInfo> detect(const cv::Mat& frame) const;

  static cv::Mat visualizeTags(const std::vector<apriltag::TagInfo>& list_tags,
                               const cv::Mat& frame);

private:
  apriltag::TagType tag_type_;
  apriltag_family_t* tag_family_;
  apriltag_detector_t* tag_detector_;
  double tag_size_;  // m

  // camera intrinsic related params
  // for more into see
  // https://docs.opencv.org/4.5.1/dc/dbb/tutorial_py_calibration.html
  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

}  // namespace object_detection

#endif /* __OBJECT_DETECTION_APRILTAG_DETECTOR_HPP__ */
