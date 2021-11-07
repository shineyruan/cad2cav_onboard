#include "object_detection/landmark_processor.hpp"

namespace object_detection {

void LandmarkProcessor::init() {
  img_sub_ = it_.subscribe("/camera/image_raw", 1,
                           &LandmarkProcessor::imageReceiveCallback, this);
  ROS_WARN("Listening to topic /camera/image_raw for landmark processing...");

  landmark_pub_ =
      n_.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 1);
  ROS_INFO("Landmarks published to topic /landmark");

  initCameraParams();
  ROS_INFO("Camera params initialized");
}

LandmarkProcessor::LandmarkProcessor(const std::string model_path,
                                     const std::string config_path,
                                     DNNType dnn_type, DatasetType dataset_type,
                                     cv::String camera_calibration_params_path,
                                     std::string tag_family, double tag_size)
    : n_(ros::NodeHandle()),
      it_(n_),
      camera_params_path_(camera_calibration_params_path),
      camera_distortion_coeff_(5),
      current_frame_() {
  object_detector_ = std::make_unique<ObjectDetector>(
      model_path, dnn_type, dataset_type, config_path);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(tag_family, tag_size,
                                                          camera_intrinsic_);
}

LandmarkProcessor::LandmarkProcessor(std::string tag_family, double tag_size)
    : n_(ros::NodeHandle()),
      it_(n_),
      camera_params_path_(DEFAULT_CAM_PARAM_PATH),
      camera_distortion_coeff_(5),
      current_frame_() {
  object_detector_ =
      std::make_unique<ObjectDetector>(DEFAULT_MODEL_PATH, DNNType::DARKNET,
                                       DatasetType::COCO, DEFAULT_CONFIG_PATH);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(tag_family, tag_size,
                                                          camera_intrinsic_);
}

LandmarkProcessor::LandmarkProcessor(const std::string model_path,
                                     const std::string config_path,
                                     DNNType dnn_type, DatasetType dataset_type,
                                     std::string tag_family, double tag_size)
    : n_(ros::NodeHandle()),
      it_(n_),
      camera_params_path_(DEFAULT_CAM_PARAM_PATH),
      camera_distortion_coeff_(5),
      current_frame_() {
  object_detector_ = std::make_unique<ObjectDetector>(
      model_path, dnn_type, dataset_type, config_path);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(tag_family, tag_size,
                                                          camera_intrinsic_);
}

void LandmarkProcessor::publishLandmark(
    const std::vector<BoundingBox>& bbox_list,
    const ros::Time& msg_stamp) const {
  cartographer_ros_msgs::LandmarkList landmark_list;
  landmark_list.header.stamp    = msg_stamp;
  landmark_list.header.frame_id = "base_link";

  for (unsigned int i = 0; i < bbox_list.size(); ++i) {
    cartographer_ros_msgs::LandmarkEntry landmark;
    landmark.id                                          = std::to_string(i);
    landmark.tracking_from_landmark_transform.position.x = 0;
    landmark.tracking_from_landmark_transform.position.y = 0;
    landmark.tracking_from_landmark_transform.position.z = 0;
    landmark.rotation_weight                             = 1e6;
    landmark.translation_weight                          = 1e6;
    landmark_list.landmarks.emplace_back(landmark);
  }

  landmark_pub_.publish(landmark_list);
}

void LandmarkProcessor::publishLandmark(
    const std::vector<apriltag::TagInfo>& tag_list,
    const ros::Time& msg_stamp) const {
  cartographer_ros_msgs::LandmarkList landmark_list;
  landmark_list.header.stamp    = msg_stamp;
  landmark_list.header.frame_id = "base_link";

  for (unsigned int i = 0; i < tag_list.size(); ++i) {
    cartographer_ros_msgs::LandmarkEntry landmark;
    landmark.id = std::to_string(tag_list[i].id);
    landmark.tracking_from_landmark_transform.position.x = tag_list[i].pos[0];
    landmark.tracking_from_landmark_transform.position.y = tag_list[i].pos[1];
    landmark.tracking_from_landmark_transform.position.z = tag_list[i].pos[2];
    Eigen::Quaternionf quat_orientation{tag_list[i].rot};
    landmark.tracking_from_landmark_transform.orientation.x =
        quat_orientation.x();
    landmark.tracking_from_landmark_transform.orientation.y =
        quat_orientation.y();
    landmark.tracking_from_landmark_transform.orientation.z =
        quat_orientation.z();
    landmark.tracking_from_landmark_transform.orientation.w =
        quat_orientation.w();
    landmark.rotation_weight    = 1e6;
    landmark.translation_weight = 1e6;
    landmark_list.landmarks.emplace_back(landmark);
  }

  landmark_pub_.publish(landmark_list);
}

void LandmarkProcessor::imageReceiveCallback(
    const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  } catch (const std::exception& e) {
    ROS_ERROR("%s", e.what());
  }

  if (frame.empty()) return;

  // update frame
  updateFrame(frame);

  // visualize bounding box
  const auto detection_results = apriltag_detector_->detect(current_frame_);
  visualize(AprilTagDetector::visualizeTags(detection_results, current_frame_));

  // publish landmark to Cartographer SLAM
  publishLandmark(detection_results, msg->header.stamp);
}

void LandmarkProcessor::initCameraParams() {
  cv::FileStorage file(camera_params_path_, cv::FileStorage::READ);
  if (!file.isOpened()) {
    ROS_FATAL("Cannot open camera calibration params at %s",
              camera_params_path_.c_str());
    throw ros::Exception("camera param read error");
  }

  cv::Mat camera_intrinsic, camera_distortion_coeff, camera_extrinsic_rot,
      camera_extrinsic_trans;
  file["intrinsic"] >> camera_intrinsic;
  camera_intrinsic_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      camera_intrinsic.ptr<double>());
  ROS_INFO_STREAM("Loaded camera intrinsic:\n" << camera_intrinsic_ << "\n");

  file["distortion"] >> camera_distortion_coeff;
  camera_distortion_coeff_ =
      Eigen::Map<Eigen::RowVectorXd>(camera_distortion_coeff.ptr<double>(), 5);
  ROS_INFO_STREAM("Loaded camera distortion:\n"
                  << camera_distortion_coeff_ << "\n");

  file["rotation"] >> camera_extrinsic_rot;
  camera_extrinsic_rot_ =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          camera_extrinsic_rot.ptr<double>());
  ROS_INFO_STREAM("Loaded camera extrinsic rotation matrix:\n"
                  << camera_extrinsic_rot_ << "\n");

  file["translation"] >> camera_extrinsic_trans;
  camera_extrinsic_trans_ =
      Eigen::Map<Eigen::Vector3d>(camera_extrinsic_trans.ptr<double>());
  ROS_INFO_STREAM("Loaded camera extrinsic translation matrix:\n"
                  << camera_extrinsic_trans_ << "\n");
}

}  // namespace object_detection