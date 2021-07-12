#include "object_detection/apriltag_detector.hpp"

namespace object_detection {

AprilTagDetector::AprilTagDetector(apriltag::TagType tag_type, double tag_size,
                                   double fx, double fy, double cx, double cy)
    : tag_type_(tag_type),
      tag_family_(nullptr),
      tag_detector_(apriltag_detector_create()),
      tag_size_(tag_size),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy) {
  switch (tag_type) {
    case apriltag::TagType::TAG_36H11:
      tag_family_ = tag36h11_create();
      break;
    case apriltag::TagType::TAG_STANDARD41H12:
      tag_family_ = tagStandard41h12_create();
      break;
    case apriltag::TagType::TAG_STANDARD52H13:
      tag_family_ = tagStandard52h13_create();
      break;
    case apriltag::TagType::TAG_CIRCLE21H7:
      tag_family_ = tagCircle21h7_create();
      break;
    case apriltag::TagType::TAG_CIRCLE49H12:
      tag_family_ = tagCircle49h12_create();
      break;
    case apriltag::TagType::TAG_CUSTOM48H12:
      tag_family_ = tagCustom48h12_create();
      break;

    default:
      break;
  }

  if (tag_detector_ && tag_family_)
    apriltag_detector_add_family(tag_detector_, tag_family_);
}

AprilTagDetector::AprilTagDetector(apriltag::TagType tag_type, double tag_size,
                                   const cv::Mat& intrinsic_mtx)
    : AprilTagDetector(tag_type, tag_size, intrinsic_mtx.at<double>(0, 0),
                       intrinsic_mtx.at<double>(1, 1),
                       intrinsic_mtx.at<double>(0, 2),
                       intrinsic_mtx.at<double>(1, 2)) {}

AprilTagDetector::AprilTagDetector(const std::string tag_type_str,
                                   double tag_size,
                                   const cv::Mat& intrinsic_mtx)
    : AprilTagDetector(apriltag::fromString(tag_type_str), tag_size,
                       intrinsic_mtx) {}

AprilTagDetector::~AprilTagDetector() {
  // tag family clean up
  if (tag_family_) {
    switch (tag_type_) {
      case apriltag::TagType::TAG_36H11:
        tag36h11_destroy(tag_family_);
        break;
      case apriltag::TagType::TAG_STANDARD41H12:
        tagStandard41h12_destroy(tag_family_);
        break;
      case apriltag::TagType::TAG_STANDARD52H13:
        tagStandard52h13_destroy(tag_family_);
        break;
      case apriltag::TagType::TAG_CIRCLE21H7:
        tagCircle21h7_destroy(tag_family_);
        break;
      case apriltag::TagType::TAG_CIRCLE49H12:
        tagCircle49h12_destroy(tag_family_);
        break;
      case apriltag::TagType::TAG_CUSTOM48H12:
        tagCustom48h12_destroy(tag_family_);
        break;

      default:
        break;
    }
  }

  // tag detector clean up
  if (tag_detector_) apriltag_detector_destroy(tag_detector_);
}

std::vector<apriltag::TagInfo> AprilTagDetector::detect(
    const cv::Mat& frame) const {
  static cv::Mat grayscale_frame;
  cv::cvtColor(frame, grayscale_frame, cv::COLOR_BGR2GRAY);

  // prepare img header for apriltag detection
  image_u8_t img = {.width  = grayscale_frame.cols,
                    .height = grayscale_frame.rows,
                    .stride = grayscale_frame.cols,
                    .buf    = grayscale_frame.data};
  // detect apriltag
  zarray_t* detections = apriltag_detector_detect(tag_detector_, &img);

  // process data
  std::vector<apriltag::TagInfo> list_tags;
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    apriltag_detection_info_t det_info = {.det     = det,
                                          .tagsize = tag_size_,
                                          .fx      = fx_,
                                          .fy      = fy_,
                                          .cx      = cx_,
                                          .cy      = cy_};
    apriltag_pose_t tag_pose;
    estimate_tag_pose(&det_info, &tag_pose);

    apriltag::TagInfo tag;
    tag.id   = det->id;
    tag.type = tag_type_;
    list_tags.emplace_back(tag);
  }

  return list_tags;
}

}  // namespace object_detection