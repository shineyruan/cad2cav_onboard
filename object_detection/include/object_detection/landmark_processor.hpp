#ifndef __OBJECT_DETECTION_LANDMARK_PROCESSOR_HPP__
#define __OBJECT_DETECTION_LANDMARK_PROCESSOR_HPP__

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cartographer_ros_msgs/LandmarkList.h"
#include "object_detection/apriltag_detector.hpp"
#include "object_detection/object_detector.hpp"

namespace object_detection {

class LandmarkProcessor {
public:
  LandmarkProcessor(std::string tag_family = "TagStandard41h12",
                    double tag_size        = 0.01495);
  LandmarkProcessor(const std::string model_path, const std::string config_path,
                    DNNType dnn_type, DatasetType dataset_type,
                    std::string tag_family = "TagStandard41h12",
                    double tag_size        = 0.01495);
  LandmarkProcessor(const std::string model_path, const std::string config_path,
                    DNNType dnn_type, DatasetType dataset_type,
                    cv::String camera_calibration_params_path,
                    std::string tag_family = "TagStandard41h12",
                    double tag_size        = 0.01495);

  cv::Mat getFrame() const { return current_frame_; }

  void updateFrame(const cv::Mat& frame) { current_frame_ = frame; }

  void visualize(const cv::Mat& frame) const {
    cv::imshow("received image message", frame);
    cv::waitKey(1);
  }

  void publishLandmark(const std::vector<BoundingBox>& bbox_list,
                       const ros::Time& msg_stamp) const;
  void publishLandmark(const std::vector<apriltag::TagInfo>& tag_list,
                       const ros::Time& msg_stamp) const;

  void imageReceiveCallback(const sensor_msgs::ImageConstPtr& msg);

private:
  const cv::String PACKAGE_DIR = ros::package::getPath("object_detection");
  const cv::String DEFAULT_MODEL_PATH = cv::utils::fs::join(
                       PACKAGE_DIR, "models/yolov4-tiny.weights"),
                   DEFAULT_CONFIG_PATH = cv::utils::fs::join(
                       PACKAGE_DIR, "models/yolov4-tiny.cfg");
  const cv::String DEFAULT_CAM_PARAM_PATH =
      cv::utils::fs::join(PACKAGE_DIR, "params/camera/params.xml");

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Publisher landmark_pub_;

  cv::String camera_params_path_;
  cv::Mat camera_intrinsic_;
  cv::Mat camera_distortion_coeff_;
  cv::Mat camera_extrinsic_rot_;
  cv::Mat camera_extrinsic_trans_;

  cv::Mat current_frame_;
  std::unique_ptr<ObjectDetector> object_detector_;
  std::unique_ptr<AprilTagDetector> apriltag_detector_;

  void init();
  void initCameraParams();
};

}  // namespace object_detection

#endif /* __OBJECT_DETECTION_LANDMARK_PROCESSOR_HPP__ */
