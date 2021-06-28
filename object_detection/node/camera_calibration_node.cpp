#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "object_detection/video_loader.hpp"

int main(int argc, char const *argv[]) {
  constexpr int CHECKBOARD_XMAX = 8, CHECKBOARD_YMAX = 6;
  constexpr double CHECKBOARD_L = 0.035;  // measured in m

  // object points in 3D world space
  std::vector<std::vector<cv::Point3d>> obj_points;

  // define world coordinates for world object points
  std::vector<cv::Point3d> obj_points_per_checkerboard;
  for (int i = 0; i < CHECKBOARD_YMAX; ++i)
    for (int j = 0; j < CHECKBOARD_XMAX; ++j)
      obj_points_per_checkerboard.push_back(
          cv::Point3d(j * CHECKBOARD_L, i * CHECKBOARD_L, 0.));

  // open webcam
  unsigned int webcam_idx = 0;
  object_detection::VideoLoader video_loader(std::to_string(webcam_idx), true);

  while (video_loader.nextFrame()) {
    auto frame = video_loader.getFrame();
    cv::Mat frame_grayscale;
    cv::cvtColor(frame, frame_grayscale, cv::COLOR_BGR2GRAY);

    // find checkerboard corners
    std::vector<cv::Point2d> corner_pts;
    bool success = cv::findChessboardCorners(
        frame_grayscale, cv::Size(CHECKBOARD_XMAX, CHECKBOARD_YMAX), corner_pts,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
            cv::CALIB_CB_NORMALIZE_IMAGE);
    if (success) {
      ROS_INFO("Chessboard corners found successful. Refining subpixels...");
      cv::TermCriteria criteria(
          cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(frame_grayscale, corner_pts, cv::Size(11, 11),
                       cv::Size(-1, -1), criteria);
      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame,
                                cv::Size(CHECKBOARD_XMAX, CHECKBOARD_YMAX),
                                corner_pts, success);
    }

    video_loader.visualize(frame);
  }

  // image points in pixel coordinates
  std::vector<std::vector<cv::Point2d>> img_points;

  return 0;
}
