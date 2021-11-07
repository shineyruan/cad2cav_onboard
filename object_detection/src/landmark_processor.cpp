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
      current_frame_() {
  object_detector_ = std::make_unique<ObjectDetector>(
      model_path, dnn_type, dataset_type, config_path);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(
      tag_family, tag_size, camera_.getInfo().intrinsic_);
}

LandmarkProcessor::LandmarkProcessor(std::string tag_family, double tag_size)
    : n_(ros::NodeHandle()),
      it_(n_),
      camera_params_path_(DEFAULT_CAM_PARAM_PATH),
      current_frame_() {
  object_detector_ =
      std::make_unique<ObjectDetector>(DEFAULT_MODEL_PATH, DNNType::DARKNET,
                                       DatasetType::COCO, DEFAULT_CONFIG_PATH);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(
      tag_family, tag_size, camera_.getInfo().intrinsic_);
}

LandmarkProcessor::LandmarkProcessor(const std::string model_path,
                                     const std::string config_path,
                                     DNNType dnn_type, DatasetType dataset_type,
                                     std::string tag_family, double tag_size)
    : n_(ros::NodeHandle()),
      it_(n_),
      camera_params_path_(DEFAULT_CAM_PARAM_PATH),
      current_frame_() {
  object_detector_ = std::make_unique<ObjectDetector>(
      model_path, dnn_type, dataset_type, config_path);
  init();
  apriltag_detector_ = std::make_unique<AprilTagDetector>(
      tag_family, tag_size, camera_.getInfo().intrinsic_);
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
  cad2cav_msgs::LandmarkDetectionList landmark_list;
  landmark_list.header.stamp    = msg_stamp;
  landmark_list.header.frame_id = "base_link";

  cad2cav_msgs::LandmarkDetection landmark_detection;
  landmark_detection.camera_param = camera_.toMsg();
  for (size_t i = 0; i < tag_list.size(); ++i) {
    cad2cav_msgs::LandmarkEntry landmark;
    landmark.id                            = std::to_string(tag_list[i].id);
    landmark.pose_in_body_frame.position.x = tag_list[i].pos[0];
    landmark.pose_in_body_frame.position.y = tag_list[i].pos[1];
    landmark.pose_in_body_frame.position.z = tag_list[i].pos[2];
    Eigen::Quaternionf quat_orientation{tag_list[i].rot};
    landmark.pose_in_body_frame.orientation.x = quat_orientation.x();
    landmark.pose_in_body_frame.orientation.y = quat_orientation.y();
    landmark.pose_in_body_frame.orientation.z = quat_orientation.z();
    landmark.pose_in_body_frame.orientation.w = quat_orientation.w();
    landmark_detection.landmark_list.push_back(std::move(landmark));
  }

  landmark_list.landmark_detections.push_back(std::move(landmark_detection));
  landmark_pub_.publish(landmark_list);
}

void LandmarkProcessor::publishLandmarkToCartographer(
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

  cad2cav::CameraInfo camera_info;

  cv::Mat camera_intrinsic, camera_distortion_coeff, camera_extrinsic_rot,
      camera_extrinsic_trans;
  file["intrinsic"] >> camera_intrinsic;
  camera_info.intrinsic_ =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          camera_intrinsic.ptr<double>());
  ROS_INFO_STREAM("Loaded camera intrinsic:\n"
                  << camera_info.intrinsic_ << "\n");

  file["distortion"] >> camera_distortion_coeff;
  Eigen::RowVectorXd eigen_camera_distortion_coeff{5};
  eigen_camera_distortion_coeff =
      Eigen::Map<Eigen::RowVectorXd>(camera_distortion_coeff.ptr<double>(), 5);
  ROS_INFO_STREAM("Loaded camera distortion:\n"
                  << eigen_camera_distortion_coeff << "\n");
  for (size_t i = 0; i < camera_info.distortion_coeff_.size(); ++i) {
    camera_info.distortion_coeff_.at(i) = eigen_camera_distortion_coeff[i];
  }

  file["rotation"] >> camera_extrinsic_rot;
  Eigen::Matrix3d eigen_camera_extrinsic_rot =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          camera_extrinsic_rot.ptr<double>());

  file["translation"] >> camera_extrinsic_trans;
  Eigen::Vector3d eigen_camera_extrinsic_trans =
      Eigen::Map<Eigen::Vector3d>(camera_extrinsic_trans.ptr<double>());

  camera_info.extrinsic_.block<3, 3>(0, 0) = eigen_camera_extrinsic_rot;
  camera_info.extrinsic_.block<3, 1>(0, 3) = eigen_camera_extrinsic_trans;

  ROS_INFO_STREAM("Loaded camera extrinsic matrix:\n"
                  << camera_info.extrinsic_ << "\n");

  camera_.setInfo(camera_info);
}

}  // namespace object_detection