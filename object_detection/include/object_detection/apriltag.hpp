#ifndef __APRILTAG_HPP__
#define __APRILTAG_HPP__

/**
 * @file apriltag.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 *
 * @brief Implements C++ basic data structs wrapper for Apriltag library
 *
 * @date 2021-07-12
 */

#include <ros/console.h>

#include <opencv2/opencv.hpp>

#include "apriltag-3.1.4/apriltag.h"
#include "apriltag-3.1.4/apriltag_pose.h"
#include "apriltag-3.1.4/tag36h11.h"
#include "apriltag-3.1.4/tagCircle21h7.h"
#include "apriltag-3.1.4/tagCircle49h12.h"
#include "apriltag-3.1.4/tagCustom48h12.h"
#include "apriltag-3.1.4/tagStandard41h12.h"
#include "apriltag-3.1.4/tagStandard52h13.h"

namespace object_detection {
namespace apriltag {

namespace utils {
inline cv::Vec3f matdToAxisAngle(matd_t* R) {
  assert(R->nrows == 3 && R->ncols == 3);
  const cv::Mat rotation_mat = cv::Mat(R->nrows, R->ncols, CV_64F, R->data);
  cv::Mat rotation_vec;
  cv::Rodrigues(rotation_mat, rotation_vec);
  rotation_vec.convertTo(rotation_vec, CV_32F);
  return rotation_vec.reshape(3).at<cv::Vec3f>();
}
}  // namespace utils

enum class TagType {
  TAG_36H11 = 0,
  TAG_STANDARD41H12,
  TAG_STANDARD52H13,
  TAG_CIRCLE21H7,
  TAG_CIRCLE49H12,
  TAG_CUSTOM48H12,
  UNKNOWN
};

inline TagType fromString(std::string tag_type) {
  if (tag_type.empty()) {
    ROS_ERROR("Empty Apriltag type");
    return TagType::UNKNOWN;
  }

  if (tag_type == "Tag36h11")
    return TagType::TAG_36H11;
  else if (tag_type == "TagStandard41h12")
    return TagType::TAG_STANDARD41H12;
  else if (tag_type == "TagStandard52h13")
    return TagType::TAG_STANDARD52H13;
  else if (tag_type == "TagCircle21h7")
    return TagType::TAG_CIRCLE21H7;
  else if (tag_type == "TagCircle49h12")
    return TagType::TAG_CIRCLE49H12;
  else if (tag_type == "TagCustom48h12")
    return TagType::TAG_CUSTOM48H12;
  else {
    ROS_ERROR_STREAM("Unknown Apriltag type: " << tag_type);
    return TagType::UNKNOWN;
  }
}

struct TagInfo {
  int id;         // id
  TagType type;   // tag type
  cv::Vec3f pos;  // position in camera frame
  cv::Vec3f rot;  // rotation in camera frame in axis-angle representation

  // tag location in image pixel coordinates
  cv::Point2f tag_center;                  // center of the tag
  std::array<cv::Point2f, 4> tag_corners;  // corners of the tag;
                                           // always counter-clockwise

  void setPos(matd_t* t) {
    pos = cv::Vec3f(t->data[0], t->data[1], t->data[2]);
  }

  void setRot(matd_t* R) { rot = utils::matdToAxisAngle(R); }
};

}  // namespace apriltag
}  // namespace object_detection

#endif /* __APRILTAG_HPP__ */
