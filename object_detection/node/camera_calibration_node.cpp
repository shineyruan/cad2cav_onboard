#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "object_detection/video_loader.hpp"

int main(int argc, char const* argv[]) {
  constexpr int CHECKBOARD_XMAX = 8;      // column
  constexpr int CHECKBOARD_YMAX = 6;      // row
  constexpr double CHECKBOARD_L = 0.035;  // length of unit of checkerboard.
                                          // measured in m
  const std::string img_save_dir = "camera";

  ROS_WARN(
      "Starting camera calibration. Please make sure the checkboard is placed "
      "at exactly 30cm from the camera for the last captured image.");

  // object points in 3D world space
  std::vector<std::vector<cv::Point3f>> obj_points;

  std::vector<cv::Point3f> obj_points_2d;
  for (int i = 0; i < CHECKBOARD_YMAX; ++i)
    for (int j = 0; j < CHECKBOARD_XMAX; ++j)
      obj_points_2d.push_back(
          cv::Point3f(j * CHECKBOARD_L, i * CHECKBOARD_L, 0.));

  // image points in pixel coordinates
  std::vector<std::vector<cv::Point2f>> img_points;

  // open webcam
  unsigned int webcam_idx = 0;
  object_detection::VideoLoader video_loader(std::to_string(webcam_idx), true);

  while (video_loader.nextFrame()) {
    auto frame = video_loader.getFrame();
    cv::Mat frame_grayscale;
    cv::cvtColor(frame, frame_grayscale, cv::COLOR_BGR2GRAY);

    // find checkerboard corners
    std::vector<cv::Point2f> corner_pts;
    bool success = cv::findChessboardCorners(
        frame_grayscale, cv::Size(CHECKBOARD_XMAX, CHECKBOARD_YMAX), corner_pts,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
            cv::CALIB_CB_NORMALIZE_IMAGE);
    if (success) {
      cv::TermCriteria criteria(
          cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(frame_grayscale, corner_pts, cv::Size(11, 11),
                       cv::Size(-1, -1), criteria);
      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame,
                                cv::Size(CHECKBOARD_XMAX, CHECKBOARD_YMAX),
                                corner_pts, success);
      obj_points.push_back(obj_points_2d);
      img_points.push_back(corner_pts);
    }

    video_loader.visualize(frame);
    video_loader.saveFrameToFile(img_save_dir);
  }

  // camera intrinsic calibration
  //  extrinsic params obtained here is not accurate; will replace with real
  //  world object points in the next section
  ROS_INFO("Detected %ld object points, %ld image points", obj_points.size(),
           img_points.size());
  cv::Mat intrinsic_mtx, distortion_coeff, rotation_vec, translation_vec;
  cv::calibrateCamera(
      obj_points, img_points,
      cv::Size(video_loader.getFrame().rows, video_loader.getFrame().cols),
      intrinsic_mtx, distortion_coeff, rotation_vec, translation_vec);
  ROS_INFO("----- Camera intrinsic calibration finished -----");

  // camera extrinsic calibration
  // define world coordinates for world object points
  //  use coordinate the same as body frame. X forward, Y left, Z up
  std::vector<cv::Point3f> obj_points_per_checkerboard;
  for (int i = 0; i < CHECKBOARD_YMAX; ++i)
    for (int j = 0; j < CHECKBOARD_XMAX; ++j)
      obj_points_per_checkerboard.push_back(
          cv::Point3f(0.3, (CHECKBOARD_XMAX / 2 - j) * CHECKBOARD_L,
                      (CHECKBOARD_YMAX / 2 - i) * CHECKBOARD_L));

  cv::solvePnP(obj_points_per_checkerboard, img_points.back(), intrinsic_mtx,
               distortion_coeff, rotation_vec, translation_vec, false,
               cv::SOLVEPNP_IPPE);
  ROS_INFO("----- Camera extrinsic calibration finished -----");

  cv::Mat rotation_mat;
  cv::Rodrigues(rotation_vec, rotation_mat);

  ROS_INFO_STREAM("Camera intrinsic matrix:\n" << intrinsic_mtx << "\n");
  ROS_INFO_STREAM("Distortion coefficients:\n" << distortion_coeff << "\n");
  ROS_INFO_STREAM("Rotation matrix:\n" << rotation_mat << "\n");
  ROS_INFO_STREAM("Translation vector:\n" << translation_vec << "\n");

  // save calibrated data to file
  cv::FileStorage file(cv::format("%s/params.xml", img_save_dir.c_str()),
                       cv::FileStorage::WRITE);
  file << "intrinsic" << intrinsic_mtx << "distortion" << distortion_coeff
       << "rotation" << rotation_mat << "translation" << translation_vec;
  file.release();

  return 0;
}
