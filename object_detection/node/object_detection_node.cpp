#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cartographer_ros_msgs/LandmarkList.h"
#include "object_detection/object_detector.hpp"

namespace object_detection {

class ImageReceiver {
public:
  ImageReceiver()
      : n_(ros::NodeHandle()),
        it_(n_),
        current_frame_(),
        object_detector_(DEFAULT_MODEL_PATH, object_detection::DNNType::DARKNET,
                         object_detection::DatasetType::COCO,
                         DEFAULT_CONFIG_PATH) {
    img_sub_ = it_.subscribe("/camera/image_raw", 1,
                             &ImageReceiver::imageReceiveCallback, this);
    landmark_pub_ =
        n_.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 1);
  }

  ImageReceiver(const std::string model_path, const std::string config_path,
                DNNType dnn_type, DatasetType dataset_type)
      : n_(ros::NodeHandle()),
        it_(n_),
        current_frame_(),
        object_detector_(model_path, dnn_type, dataset_type, config_path) {
    img_sub_ = it_.subscribe("/camera/image_raw", 1,
                             &ImageReceiver::imageReceiveCallback, this);
    landmark_pub_ =
        n_.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 1);
  }

  cv::Mat getFrame() const { return current_frame_; }

  void updateFrame(const cv::Mat& frame) { current_frame_ = frame; }

  void visualize(const cv::Mat& frame) const {
    cv::imshow("received image message", frame);
    cv::waitKey(1);
  }

  void publishLandmark(const std::vector<BoundingBox>& bbox_list,
                       const ros::Time& msg_stamp) const {
    cartographer_ros_msgs::LandmarkList landmark_list;
    landmark_list.header.stamp    = msg_stamp;
    landmark_list.header.frame_id = "base_link";

    for (unsigned int i = 0; i < bbox_list.size(); ++i) {
      cartographer_ros_msgs::LandmarkEntry landmark;
      landmark.id                                          = std::to_string(i);
      landmark.tracking_from_landmark_transform.position.x = 10;
      landmark.tracking_from_landmark_transform.position.y = 0;
      landmark.tracking_from_landmark_transform.position.z = 0;
      landmark.rotation_weight                             = 1e6;
      landmark.translation_weight                          = 1e6;
      landmark_list.landmarks.emplace_back(landmark);
    }

    landmark_pub_.publish(landmark_list);
  }

  void imageReceiveCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    } catch (const std::exception& e) {
      ROS_ERROR("%s", e.what());
    }

    if (frame.empty()) return;

    // update frame
    updateFrame(frame);

    // visualize bounding box
    const auto detection_results = object_detector_.infer(current_frame_);
    visualize(ObjectDetector::visualizeBBox(current_frame_, detection_results));

    // publish landmark to Cartographer SLAM
    publishLandmark(detection_results, msg->header.stamp);
  }

private:
  const cv::String package_dir = ros::package::getPath("object_detection");
  const cv::String DEFAULT_MODEL_PATH = cv::utils::fs::join(
                       package_dir, "models/yolov4-tiny.weights"),
                   DEFAULT_CONFIG_PATH = cv::utils::fs::join(
                       package_dir, "models/yolov4-tiny.cfg");

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Publisher landmark_pub_;

  cv::Mat current_frame_;
  object_detection::ObjectDetector object_detector_;
};

}  // namespace object_detection

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection_node");

  object_detection::ImageReceiver img_receiver;
  ros::spin();

  return 0;
}
